function [satmod_c, satmod_dist] = lin_sat_wheels_mod(Is, ws, hw)

Aw(:,1) = [0, Is(3,1), Is(2,1); -2*Is(3,1), -Is(3,2), -Is(3,3)+Is(1,1); 2*Is(2,1), Is(2,2)-Is(1,1), Is(2,3)]*ws;
Aw(:,2) = [Is(3,1), 2*Is(3,2), (Is(3,3)-Is(2,2)); -Is(3,2), 0, Is(1,2); Is(2,2)-Is(1,1), 2*Is(1,2),-Is(1,3)]*ws;
Aw(:,3)= [-Is(2,1),Is(3,3)-Is(2,2),-2*Is(2,3); Is(1,1)-Is(3,3), Is(1,2), 2*Is(1,3); Is(2,3), -Is(1,3), 0]*ws ;
invrtIs = inv(Is);
Ahw = [0, -hw(3), hw(2); hw(3), 0, -hw(1); -hw(2), hw(1), 0];
Awh = [0, ws(3), -ws(2); -ws(3), 0, ws(1); ws(2), -ws(1), 0];
Aww = Aw + Ahw;
A = [invrtIs*Aww, zeros(3,3), invrtIs*Awh; 0.5*eye(3,3), zeros(3,3), zeros(3,3); zeros(3,3), zeros(3,3), zeros(3,3)];
Bu = [invrtIs; zeros(3,3); -eye(3,3)];
Bd = [invrtIs; zeros(3,3); zeros(3,3)];
C = [eye(6,6), zeros(6,3)]; D = zeros(6,3);
satmod_c = ss(A,Bu,C,D); satmod_dist = ss(A,Bd,C,D);