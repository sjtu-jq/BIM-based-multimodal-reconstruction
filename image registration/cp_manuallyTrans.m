function [I1_aff,affmat] = cp_manuallyTrans(I1,I2)
%CP_MANUALLYTRANS obtain transformation matrix by manually marking
figure,
subplot(121), imshow(I1);
subplot(122), imshow(I2);

figure,
imshow(I1);
%%
title('选取红外配准点');
[uL,vL,~]=impixel(I1);
hold on;
plot(uL,vL,'go'),
plot(uL,vL,'y+'),
for i=1:length(uL)
    text(uL(i),vL(i),num2str(i),'Color','w');
end
hold off;

figure,
title('选取可见光配准点'),
[uR,vR,~]=impixel(I2); 
hold on,
plot(uR,vR,'go');
plot(uR,vR,'y+');
for i=1:length(uL)
    text(uR(i),vR(i),num2str(i),'Color','y');
end
hold off;
%%
p1 = [uL vL]; p2 = [uR vR];
p3 = cp_subpixelFine(p1,p2); 
[I1_aff,affmat] = cp_getAffine(I1,I2,p1,p3);
end

