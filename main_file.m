%clear workspace
clc
clear all

%Coordinates of MAE
 MAE = [[-1.25 0.5 0];[-1.25 1.5 0];[-0.75 1 0];[-0.25 1.5  0];[-0.25 0.5 0];
      [0.25 1.5 0];[0.75 0.5 0];[0.5 1 0];[0 1 0];[0.51 1 0];[0.751 0.5 0];
      [1.25 0.5 0];[0.751 0.5 0];[0.75 1.5 0];[1.25 1.5 0];[0.751 1.5 0];
      [0.75 1 0];[1.25 1 0]];

%Coordinates of 547
 NUM = [[-0.75 -0.25 0];[-1.25 -0.25 0];[-1.25 -0.75 0];[-0.75 -0.75 0];[-0.75 -1.25 0];
      [-1.25 -1.25 0];[-0.25 -0.25 0];[-0.25 -0.75 0];[0.25 -0.75 0];[0.25 -0.5 0];[0.25 -1.25 0];
      [0.75 -0.25 0];[1.25 -0.25 0];[1 -0.75 0];[1 -1.25 0]];

%Define Planner robot  
 mdl_planar2

%Define Q matrix
 Q = zeros(5,2);

%plot for MAE 
for i = 1:17   %for loop 
    hold on
        plot_arrow(MAE(i,:),MAE(i+1,:),'headwidth',0.25);   %creat arrow for MAE
    hold off
    x = mtraj (@lspb, MAE(i,:),MAE(i+1,:),20);   %Divide in 20 parts

    for j = 1:18
        Px = x(j,1);    %extract 1st column
        Py = x(j,2);    %extract 2nd column                    
        T = [1 0 0 Px; 0 1 0 Py; 0 0 1 0; 0 0 0 1];  
        Q(j,:) = p2.ikine (T,'mask',[1 1 0 0 0 0]); %Inverse code
        p2.plot(Q(j,:),'top')   %Plot the MAE
    end 
end

%Plot trajectory from last point of MAE to first point of 547
tr1 = [1 0 0 1.25; 0 1 0 1; 0 0 1 0; 0 0 0 1];  
tr2 = [1 0 0 -0.75; 0 1 0 -0.25; 0 0 1 0; 0 0 0 1];
q1 = p2.ikine (tr1,'mask',[1 1 0 0 0 0]);
q2 = p2.ikine (tr2,'mask',[1 1 0 0 0 0]);
q = jtraj(q1,q2,20);  %Plan trajectory
for i = 1:20
    p2.plot(q(i,:))   %Plot graph
end

%Plot Arrow for 547
for i = 1:14
     for j = 1:5
         hold on
             plot_arrow(NUM(j,:),NUM(j+1,:),'headwidth',0.25) %PLot 5
         hold off
     end   
     for j = 7:10
         hold on
             plot_arrow(NUM(j,:),NUM(j+1,:),'headwidth',0.25) %Plot 4
         hold off
     end   
     for j = 12:14
         hold on
             plot_arrow(NUM(j,:),NUM(j+1,:),'headwidth',0.25) %Plot 7
         hold off
     end   
     x = mtraj (@lspb, NUM(i,:),NUM(i+1,:),20); %Divide in 20 parts

     %For loop for trajectory of 547
      for k = 1:14
          Px = x(k,1);
          Py = x(k,2);
          T = [1 0 0 Px; 0 1 0 Py; 0 0 1 0; 0 0 0 1];  
          Q(k,:) = p2.ikine(T,'mask',[1 1 0 0 0 0]);   % Angle
          p2.plot(Q(k,:),'top')  %Plot the Trajectory
      end 
end

