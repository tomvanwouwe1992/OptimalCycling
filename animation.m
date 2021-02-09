function [] = animation(P_J_leg1,P_J_leg2,P_J_upperbody,dt)
figure(10)

Pxcrank1 = [P_J_leg1(:,[1 3 ])];
Pycrank1 = [P_J_leg1(:,[2 4 ])]; 

Pxcrank2 = [P_J_leg2(:,[1 3 ])];
Pycrank2 = [P_J_leg2(:,[2 4 ])]; 

Px1 = [P_J_leg1(:,[ 3 5 7 9 ])];
Py1 = [P_J_leg1(:,[ 4 6 8 10])]; 

Px2 = [P_J_leg2(:,[ 3 5 7 9 ])];
Py2 = [P_J_leg2(:,[ 4 6 8 10])];

Px3 = [P_J_upperbody(:,[1 3 5 7 9 ])];
Py3 = [P_J_upperbody(:,[2 4 6 8 10])];

H3=line(Pxcrank1(1,:), Pycrank1(1,:),'Color','k','LineWidth',2); hold on
H4=line(Pxcrank2(1,:), Pycrank2(1,:),'Color','k','LineWidth',2); hold on
Hl=line(Px1(1,:), Py1(1,:),'Color','b'); hold on
H2=line(Px2(1,:), Py2(1,:),'Color','r'); hold on
H5=line(Px3(1,:), Py3(1,:),'Color','b'); hold on

axis equal
Hl=handle(Hl);
% Hl.Color='r';
 xlim([-0.6 0.6])
 ylim([-0.5 1.7])

     movieCt = 1;
       
for i = 1:3
for j=1:size(P_J_leg1,1)
    H3.XData=Pxcrank1(j,:);
    H3.YData=Pycrank1(j,:);
    H4.XData=Pxcrank2(j,:);
    H4.YData=Pycrank2(j,:);
    Hl.XData=Px1(j,:);
    Hl.YData=Py1(j,:);
    H2.XData=Px2(j,:);
    H2.YData=Py2(j,:);  
    H5.XData=Px3(j,:);
    H5.YData=Py3(j,:);  
    Avideo(movieCt) = getframe(gcf);
    movieCt = movieCt + 1;
%     if mod(j,4) == 0 || j ==1 
%         scatter(Px1(j,1),Py1(j,1),1,'b');
%         scatter(Px1(j,2),Py1(j,2),1,'b');
%         scatter(Px1(j,4),Py1(j,4),1,'g');
%     end
end
%     Px1_next = Px2;
%     Py1_next = Py2;
%     Px2 = Px1;
%     Py2 = Py1;
%     Px1 = Px1_next;
%     Py1 = Py1_next;
end

    v = VideoWriter('video2Dgait_1.avi');
    open(v)
    writeVideo(v,Avideo)