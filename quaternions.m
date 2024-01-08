% Pt research
% Paper1: Reprezintare axis-ang prin unitate
% x: cos(theta/2) + i sin(theta/2)
% y: cos(theta/2) + j sin(theta/2)
% z: cos(theta/2) + k sin(theta/2)

% folosim ca notatii:
%   Zi = axa nr. i, in jurul careia apare rotatia
%   Oi = unghiul de rotate in jurul axei i (ar trebui sa fie theta, dar na)
%   Xi = o axa imaginara, suport pentru cupla (se roteste, dar nu apare rot
%       in jurul ei
%   Ai = unghiul relativ dintre axa Zi si axa Zi-1, dupa axa Xi
%   Li = distanta dintre Zi-1 si Zi

% allegedly, o sa restrictionam oarecum domeniul pe care se plimba Oi,
% intre -90 si +90 (ca, realist vorbind, sansele sa mergem pe alte unghiuri
% in practica este una mica). 
% Later edit: nu restrictionezi THETA, CI RESTRICTIONEZI Ai. ca foarte
% rareori vei avea un loc in care sa ai un robot cu cuplele dispuse la
% vreun unghi exotic. Tre sa vad cum au ajuns la relatiile date in tab.2,
% sa ma prind ce facem in cazul de unghi exotic

% mai este de notat ca lucrarea merge pe un manipulator nebun rau,
% cu 6 axe, dispuse Y1P2P3P4Y5

% se mai fac urmatoarele conventii:
%   Axele 1 si 2 coincid ca origine, deci L1 = 0
%   A1 = 90 (le dau si eu in grade, ca, na, mieleneeee). 
%   A2 = 0  (sunt coiliniare)
%   A3 = 0
%   A4 = 90
%   A5 = 90

% Se merge pe ideea de quaternion de translatie (Qt, parte de translatie,
% de forma (0, xi, yj, zk) ) si parte re rotatie, cu varianta clasica:
% cuaternionul devine, astfel: Qt + Q * r * Q(conj). De vazut de ce miezeu
% e asa, ca in teorie nu reorientam sistemul de coordonate in jurul
% punctului, dar cine sunt eu sa ma pun in fata fericirii matematicii

% Si spun baietii in felu' urmator:
%   r0 = x0i0 + y0j0 + z0k0 =
%      = (x1*cos(theta1)+z1*sin(theta1)) * i0
%      + (x1*sin(theta1)-z1*cos(theta1)) * j0 
%      + z1                              * k0

% STAI BA! CA M-AM LAMURIT (OARECUM)
% Mai zice baietii in felu urmator:
% pe Ai restrictionate pentru -90,0 si 90, putem descrie urmatoarele
% relatii ce ne arat modul in care se transforma axele intre ele, de la o
% cupla la alta:
%   Scriem fiecare vector de pozitie r(i-1) ca functie de r(i) (descriem
%   cupla ca depinzand de cupla urmatoare)
%   Pentru 0deg:
%       [(xi+ai)cos(Oi)-Yi*sin(Oi)] * Ii-1 + 
%       [yicos(Oi)+(xi+ai)*sin(Oi)] * Ji-1 +
%       ZiKi-1
% 
%   Pentru 90deg:
%       [(xi+ai)cos(Oi)+Zi*sin(Oi)] * Ii-1 + 
%       [yicos(Oi)+(xi+ai)*sin(Oi)] * Ji-1 +
%       ZiKi-1


% Hai sa vedem daca avem capacitatea neuronala sa vedem de unde provine
% asta (nu o avem)
% 

% de unde pana unde? Pai, ne zic baietii, in fapt, urmatoarele:
%   Transformarea de la axa 0 la axa 1


clear
rotation = rpy(0, deg2rad(45), 0);
% disp(rotation)
% disp(conj(rotation))

link1 = quaternion(0, 200, 0, 0);
link1 = conj(rotation) * link1 * rotation;
% disp(link1)

link2 = quaternion(0, 100, 0, 0);
rotation = rpy(0, deg2rad(24), 0);
link2 = conj(rotation) * link2 * rotation;
% disp(link2)

% disp(link1 + link2)

angleArr120   = [0, 0, 0, 0, 0, 0];
rotAxis120    = [3, 2, 2, 1, 2, 1];        % 1 -> r | 2 -> p | 3 -> y
                                           % r -> dupa x
                                           % p -> dupa y
                                           % y -> dupa z

% putem sa fim (si o sa fim) capcauni si sa presupunem yaw din
% baza ca fiind in efectiv podea

% distantele sunt date functie de x y z mentionate anterior (pe lungime)
% de asemenea, vorbim de deplasari relative (lungimile fiecarui segment
% suport al axei).
% de asemenea, vorbim de distantele FIZICE dintre cuple:
% distanta in linie dreapta, in sensul unei axe, de la punctul care 
% roteste pana la punctul care este rotit

% linkLength = [        % aci nu-s neaparat sigur daca e optim, insa tre
%    [0, 0, 290],...   % sa poata fi luate in calcul si dezaxarile
%    [103, 0, 0],...
%    [0]
%];

% forget about! -^ tre sa dam lungimile efective ale vectorilor de
% rotatie, adica a segmentelor. pfui, domne...
% in caz de dezaxari cringe, mutam sistemu de referinta intai si apoi
% efectuam si rotatia.
% ergo: pentru o prima modelare, facem conventia ca axa determinata de
% punctul rotitor si punctul rotit sa fie perpendiculara cu axa de rotatie

linkLengths120 = [290 + 70, 270, 134, 302 - 134, 72, 0];

linkLengthsACAD = [200, 100];
angleArrACAD    = [45, 24];
rotAxisACAD     = [2, 2];
% disp("------------------------------------------------------------");

linkAxes = [3, 1, 1, 1, 1, 1];


% from cozzy's paper: x = 374, Y = 0, Z = 630

%ee = calculateEE(angleArr120, rotAxis120, linkLengths120);
% ee = calculateEE(angleArrACAD, rotAxisACAD,linkLengthsACAD);
% disp(ee);

% e posibil ca abordarea simplista sa nu ia in calcul deplanarile, si asta
% e mai putin fun.

% 13.11: Da, nu numai că este posibil, ci așa se și întâmplă: abordarea
% simplificată nu ia în calcul deplanările și asta e nașpa.
% să zicem, însă, că am putea să facem o treabă și să rotim sistemul de axe
% întâi, ca mișcarea să se producă mereu în același plan.

% pentru aceasta, o să facem o treaba

% ee = calculateEELAXDep(angleArr120, rotAxis120, linkLengths120, linkAxes);
% disp(ee)
% disp(getRPYFromQuaternion(ee))
% disp(eulerd(ee, "XYZ", "point"));

% ee = calculateDual(angleArr120, rotAxis120, linkLengths120, linkAxes);
% [x, y, z, t] = parts(ee);
% fprintf("Final link position: %.2f %.2f %.2f %.2f\n", ...
%         x, y, z, t);

% 13.11: Ok, am o idee. Vaz ca nu e neaparat multumit daca se compun
% miscarile, si cumva are sens, pentru ca daca ne gandim, noi facem cu
% sisteme de coordonate relative unul fata de celalalt. Fals! Fals, in
% sensul ca pana acum am lucrat doar in coordonate globale. Hai sa vedem,
% intai, daca putem extrage unghiurile din rotatiile anterioare (si sa si
% aiba gen sens)

%%% 18.11 %%%

% E mai mult decat posibil ca noi sa ne fi si ales prost sistemul de
% coordonate. Deci, voi mai verifica o data:
% X in lungul robotului (in pozitia de hom)
% Y inspre stanga robotului
% Z in sus.

% vectorul de rotatii devine, deci
% nu mai devine nimic, e corect.
% vectorul de translatii devine:
% am o idee mai buna: fiecare cupla sa fie reprezentata de un punct
% material (3 coordonate carteziene, raportat la axa anterioara)

% HA! INTERESANT!
% Am facut pe figura. Avem o mica problema ce provine dintr-o dezaxare!
% Trebuie translatat sistemul de coordonate, fmm veata! Dar nu e problema.
% Rezolvam numaidecat. Problema e ca daca nu facem asta, obtinem un vector
% oblic, care nu poate da rezultatul corect pe X, desi pe Z este ok!


angleArr120   = [10, 20, 30, 40, 50, 60];
rotAxis120    = [3, 2, 2, 1, 2, 1];
axisOffsets = [ 0, 0, 290       ;  ...
                0, 0, 270       ;  ...
                134, 0, 70      ; ...
                302-134, 0, 0 ; ...
                72, 0, 0        ; ...
    ];

% since we don't care about the roll's point of action, we will translate
% axis 4 slightly rearword, eliminating the diagonal.

[ee, eeRot] = calculateDual(angleArr120, rotAxis120, axisOffsets);
[x, y, z, t] = parts(ee);
fprintf("Final link position: %.2f %.2f %.2f %.2f\n", ...
        x, y, z, t);
plotPoints = zeros(7, 3);
%inverse(ee, eeRot, axisOffsets);

function [eePos, eeRot] = calculateDual(angleArr, rotAxis, axisOffsets, plotPoints) 
    [r,c] = size(angleArr);                                  

    if ~((r > 1) || (c > 1))
        disp("this function operates on an angle array")
        EE = NaN;
        return;
    end

    finalLinkT = quaternion(0, 0, 0, 0);
    finalLinkR = quaternion(1, 0, 0, 0);
    offset = 1;
    
    for i = 1 : (c - 1)
        switch rotAxis(i)
            case 1
                rot = rpy(-deg2rad(angleArr(i)), 0, 0);
            case 2
                rot = rpy(0, -deg2rad(angleArr(i)), 0);
            case 3
                rot = rpy(0, 0, -deg2rad(angleArr(i)));
        end

        link = quaternion(0, ...
            axisOffsets(i, 1), ...
            axisOffsets(i, 2), ...
            axisOffsets(i, 3));
        
        [x, y, z, t] = parts(link);
        fprintf("At link %d: %.2f %.2f %.2f %.2f\n", i, x, y, z, t);

        rotLoc = rot * finalLinkR;
        rotated =conj(rotLoc) * link * rotLoc;

        [x, y, z, t] = parts(rotated);
        fprintf("After rotation, at link %d: %.2f %.2f %.2f %.2f\n", i, x, y, z, t);
        finalLinkT = finalLinkT + rotated;
        %finalLinkT = rotated;
        fprintf("Orientation at position is: %.2f %.2f %.2f\n", getRPYFromQuaternion(finalLinkR))

        [~, y, z, t] = parts(finalLinkT);
        plotPoints(i + offset, 1) = y;
        plotPoints(i + offset, 2) = z;
        plotPoints(i + offset, 3) = t;

        finalLinkR = rot * finalLinkR;
    end

    fprintf("Final link orientation: %.2f %.2f %.2f\n", ...
        getRPYFromQuaternion(finalLinkR));

    [x, y, z, t] = parts(finalLinkR);
    fprintf("Final rot q: %.2f, %.2f, %.2f, %.2f\n", x, y, z, t);
    
    plot3(plotPoints(:,1), plotPoints(:,2), plotPoints(:,3),'LineWidth',8, 'Marker', 'o');
    %plot(plotPoints(:,1), plotPoints(:,3), 'LineWidth',8 , 'Marker', 'o');
    %plot(plotPoints(:,1), plotPoints(:,2), 'LineWidth',8, 'Marker', 'o');
    %xlim([0, 350]);
    %ylim([0, 350]);
    
    eePos = finalLinkT;
    eeRot = finalLinkR;
end

function dispQuat(quatToDisplay)
    [x, y, z, t] = parts(quatToDisplay);

    fprintf("%.3f %.3f %.3f %.3f\n", x, y, z, t);
end

function [ax1, ax2, ax3, ax4, ax5, ax6] = inverse(posQuat, rotQuat, axisOffsets)
link = quaternion(0, ...
            axisOffsets(5, 1), ...
            axisOffsets(5, 2), ...
            axisOffsets(5, 3));
    p3 = conj(rotQuat) * link * rotQuat;
    dispQuat(posQuat);

    
    joint4 = posQuat - p3;
    dispQuat(joint4);

    link = quaternion(0, ...
            axisOffsets(4, 1), ...
            axisOffsets(4, 2), ...
            axisOffsets(4, 3));

    joint3 = posQuat - p3 - link;
    dispQuat(joint3);

    joint1 = quaternion(0, ...
            axisOffsets(1, 1), ...
            axisOffsets(1, 2), ...
            axisOffsets(1, 3));
    dispQuat(joint1);

    [~,p2,p3,p4] = parts(joint1);
    [~,q2,q3,q4] = parts(joint3);

    l3 = pdist([p2,p3,p4;q2,q3,q4], 'euclidean');
    
    l1 = pdist([0,0,0, ...
               ;axisOffsets(2,1),axisOffsets(2,2),axisOffsets(2,3)]);
    l2 = pdist([0,0,0, ...
               ;axisOffsets(3,1),axisOffsets(3,2),axisOffsets(3,3)]);

    cosb = (l1 ^ 2 + l2 ^ 2 - l3 ^ 2) / (2 * l1 * l2);
    cosa = (l2 ^ 2 + l3 ^ 2 - l1 ^ 2) / (2 * l2 * l3);
    cosc = (l3 ^ 2 + l1 ^ 2 - l2 ^ 2) / (2 * l3 * l1);
    a = rad2deg(acos(cosa));
    b = rad2deg(acos(cosb));
    c = rad2deg(acos(cosc));
    

end

function uvw = getRPYFromQuaternion(quat)

    [q0,q1,q2,q3] = parts(quat);
    v = rad2deg(asin(2*(q0*q2 - q1*q3)));
    u = rad2deg(atan2(2*(q0*q1+q2*q3), q0^2 - q1^2 - q2^2 + q3 ^ 2));
    w = rad2deg(atan2(2*(q0*q3+q1*q2), q0^2 + q1^2 - q2^2 - q3 ^ 2));

    % nota: functia asta e pacatoasa, si ar trebui evitata. Ma rog,
    % sa zicem ca pentru debugging merge, dar ar trebui sa nu.
    uvw = [u, v, w];
end

function rad = deg2rad(deg)

    rad = pi() / 180 * deg;

end

function deg = rad2deg(rad)

    deg = rad / (pi() / 180);

end

function conjugate = conj(quat)
    [q1, q2, q3, q4] = parts(quat);
    
    conjugate = quaternion(q1, -q2, -q3, -q4);

end

function r = rpy(u, v, w)
    q0 = cos(u / 2) * cos(v / 2) * cos(w / 2) + sin(u / 2) * sin(v / 2) * sin(w / 2);
    q1 = sin(u / 2) * cos(v / 2) * cos(w / 2) - cos(u / 2) * sin(v / 2) * sin(w / 2);
    q2 = cos(u / 2) * sin(v / 2) * cos(w / 2) + sin(u / 2) * cos(v / 2) * sin(w / 2);
    q3 = cos(u / 2) * cos(v / 2) * sin(w / 2) - sin(u / 2) * sin(v / 2) * cos(w / 2);
    r = quaternion(q0, q1, q2, q3);
end