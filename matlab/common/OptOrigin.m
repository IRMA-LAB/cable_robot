function vect = OptOrigin(sol0,sol1,sol2,sol3,sol4,pp0,pp1,pp2,pp3,pp4,v)

newRef = v(1:3);
newR = RotXYZ(v(4:6));
for i=1:61
    p = newRef+newR*sol0((i-1)*6+1:(i-1)*6+3);
    a = RotToTayt(newR*RotXYZ(sol0((i-1)*6+4:(i-1)*6+6)));
    pose = [p;a];
if (i==1)
vect = pose-pp0(:,i);
else
vect = [vect;pose-pp0(:,i)];
end
end

for i=1:61
    p = newRef+newR*sol1((i-1)*6+1:(i-1)*6+3);
    a = RotToTayt(newR*RotXYZ(sol1((i-1)*6+4:(i-1)*6+6)));
    pose = [p;a];

vect = [vect;pose-pp1(:,i)];
end

for i=1:61
    p = newRef+newR*sol2((i-1)*6+1:(i-1)*6+3);
    a = RotToTayt(newR*RotXYZ(sol2((i-1)*6+4:(i-1)*6+6)));
    pose = [p;a];

vect = [vect;pose-pp2(:,i)];
end

for i=1:61
    p = newRef+newR*sol3((i-1)*6+1:(i-1)*6+3);
    a = RotToTayt(newR*RotXYZ(sol3((i-1)*6+4:(i-1)*6+6)));
    pose = [p;a];

vect = [vect;pose-pp3(:,i)];
end

for i=1:61
    p = newRef+newR*sol4((i-1)*6+1:(i-1)*6+3);
    a = RotToTayt(newR*RotXYZ(sol4((i-1)*6+4:(i-1)*6+6)));
    pose = [p;a];

vect = [vect;pose-pp4(:,i)];
end

end