function [] = plotStates(t,X, params, bookkeeping,hObject)

    % unpack variables
    [x,y,phi,thLeg,dxdt,dydt,dphi_dt] = unpackState(X);
    [m,I,k,len0,g,b] = unpackParams(params); %#ok<ASGLU>
    [touchdownTime, liftoffTime] = unpackBookkeeping(bookkeeping); %#ok<ASGLU>
    [xF, lenLeg, dthLeg_dt] = calculateKinematics(X, params); 

    % plot dependent variables
%     figure;
%     rows = 1; cols = 3;
%     subplot(rows,cols,1); hold on; % stance bool
%     plot(t, isInStance(X,params));
%     title('stanceBool');
%     subplot(rows,cols,2); hold on; % xF
%     plot(t, xF);
%     title('xF');
%     subplot(rows,cols,3); hold on; % legLen
%     plot(t, lenLeg);
%     title('legLen');
handles = guidata(hObject);
axes(handles.axes2);
cla();
%     % plot states
%     figure; 
%     rows = 2; 
%     cols = 4;
%     subplot(rows,cols,1); hold on; % plot x
%     plot(t,x); title('x');
%     subplot(rows,cols,2); hold on; % plot y

%       plot(t,y); title('Height');
%       hold on;
%       line([0,6],[1,1],'Color','red');
%       line([0,6],[1.1,1.1],'Color','green');
%     subplot(rows,cols,3); hold on; % plot phi
     plot(t,phi); title('phi'); xlabel('time (s)'); ylabel('angle (rads)');
     line([2,8],[.3,.27],'Color','red');
     line([2,8],[.22,.25],'Color','green');
     handles = guidata(hObject);

     axes(handles.axes3);
    cla();
%     subplot(rows,cols,4); hold on; % plot phi
%     plot(t,thLeg); title('thLeg');
%     subplot(rows,cols,5); hold on; % plot dx
%     plot(t,dxdt); title('dx/dt');
%     subplot(rows,cols,6); hold on; % plot dy
%     plot(t,dydt); title('dy/dt');
%     subplot(rows,cols,7); hold on; % plot dphi
    plot(t,dphi_dt); title('dphi/dt'); xlabel('time(s)'); ylabel('angle/s (rads/s)');
%     subplot(rows,cols,8); hold on; % plot dthLeg
%     plot(t,dthLeg_dt); title('dthLeg/dt');
end