function varargout = singleViewModeling(varargin)
% SINGLEVIEWMODELING MATLAB code for singleViewModeling.fig
%      SINGLEVIEWMODELING, by itself, creates a new SINGLEVIEWMODELING or raises the existing
%      singleton*.
%
%      H = SINGLEVIEWMODELING returns the handle to a new SINGLEVIEWMODELING or the handle to
%      the existing singleton*.
%
%      SINGLEVIEWMODELING('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SINGLEVIEWMODELING.M with the given input arguments.
%
%      SINGLEVIEWMODELING('Property','Value',...) creates a new SINGLEVIEWMODELING or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before singleViewModeling_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to singleViewModeling_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help singleViewModeling

% Last Modified by GUIDE v2.5 06-Apr-2016 14:21:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @singleViewModeling_OpeningFcn, ...
                   'gui_OutputFcn',  @singleViewModeling_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before singleViewModeling is made visible.
function singleViewModeling_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to singleViewModeling (see VARARGIN)

% Choose default command line output for singleViewModeling
handles.output = hObject;
handles.scale = 1;
set(handles.imageView,'xtick',[]);
set(handles.imageView,'ytick',[]);
addpath('stlwrite');
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes singleViewModeling wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = singleViewModeling_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function [x,y] = getVanishingPt(color)
global plots;
disp('Set at least two lines for vanishing point')
lines = zeros(3, 0);
while 1
    disp('Click first point or press q to stop')
    [x1,y1,b] = ginput(1);    
    if b=='q'        
        break;
    end
    disp('Click second point');
    [x2,y2] = ginput(1);
    plots(size(plots,2)+1) = plot([x1 x2], [y1 y2], color,'LineWidth',2);
    lines(:, end+1) = real(cross([x1 y1 1]', [x2 y2 1]'));
end
% compute vp (3x1 vector in homogeneous coordinates)    
M = zeros(3,3);
for i = 1: size(lines,2)
    M(1,1) = M(1,1)+lines(1,i)*lines(1,i);
    M(1,2) = M(1,2)+lines(1,i)*lines(2,i);
    M(1,3) = M(1,3)+lines(1,i)*lines(3,i);
    M(2,1) = M(2,1)+lines(1,i)*lines(2,i);
    M(2,2) = M(2,2)+lines(2,i)*lines(2,i);
    M(2,3) = M(2,3)+lines(2,i)*lines(3,i);
    M(3,1) = M(3,1)+lines(1,i)*lines(3,i);
    M(3,2) = M(3,2)+lines(2,i)*lines(3,i);
    M(3,3) = M(3,3)+lines(3,i)*lines(3,i);    
end
[vp,~] = eigs(M,1,'SM');
x = vp(1)/vp(3);
y = vp(2)/vp(3);


% --- Executes on button press in startCalc.
function startCalc_Callback(hObject, eventdata, handles)
% hObject    handle to startCalc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global plots;
if ~handles.vX
    msgbox('Initial information not complete!');
elseif ~handles.refX
    msgbox('Initial information not complete!');
else
delete(plots(:));
plots = [];
% scaling factor
a_x = ((handles.vX - handles.refX) \ (handles.refX - handles.O))/handles.refXlen
a_y = ((handles.vY - handles.refY) \ (handles.refY - handles.O))/handles.refYlen
a_z = ((handles.vZ - handles.refZ) \ (handles.refZ - handles.O))/handles.refZlen

% Projection Matrix P
handles.P = [handles.vX * a_x, handles.vY * a_y, handles.vZ * a_z, handles.O];
ProjectionMatrix = handles.P %display
P  = handles.P;
P = [P(:,1) -P(:,2) -P(:,3) P(:,4)];

% Homography Matrix H
handles.Hxy = [P(:,1:2),P(:,4)];
handles.Hxz = [P(:,1),P(:,3:4)];
handles.Hyz = P(:,2:4);
Hxy = handles.Hxy
Hxz = handles.Hxz
Hyz = handles.Hyz
guidata(hObject, handles);
end

%
% --------------------------------------------------------------------
function openfile_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to openfile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global plots;
  [filename1,filepath1]=uigetfile({'*.*','All Files'},'Select File');
  if filename1
      addpath(filepath1);
      handles.g = imread(filename1);
      axes(handles.imageView);
      hold all;
      %xlim([-1 2.5*handles.width]);
      %ylim([-1 2.5*handles.height]);
      delete(get(gca,'Children'));
      imshow(handles.g);
      plots = [];
      guidata(hObject, handles);
  end

% --------------------------------------------------------------------
function savefile_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to savefile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in saveVP.
function saveVP_Callback(hObject, eventdata, handles)
% hObject    handle to saveVP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
vX = handles.vX; vY = handles.vY; vZ = handles.vZ;
save('VP.mat', 'vX', 'vY', 'vZ');
disp('vanishing points saved');

% --- Executes on button press in loadVP.
function loadVP_Callback(hObject, eventdata, handles)
% hObject    handle to loadVP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load('VP.mat', 'vX', 'vY', 'vZ');
handles.vX = vX; handles.vY = vY; handles.vZ = vZ;
disp('vanishing points loaded');
guidata(hObject, handles);

% --- Executes on button press in getOrigin.
function getOrigin_Callback(hObject, eventdata, handles)
% hObject    handle to getOrigin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global plots;
disp('Select the origin');
[x,y] = ginput(1);
plots(size(plots,2)+1) = plot(x,y,'o');
handles.O = [x;y;1];
guidata(hObject, handles);


function transformHelper(invH, im, height, width)
% maketform - imtransform / projective2d - imwarp
%im = imcrop(im);
invH = invH';
tform = projective2d(invH);
B = imwarp(im, tform);
warning('off', 'Images:initSize:adjustingMag');
figure,[I2, rect] = imcrop(B); 

delete(get(gca,'Children'));
imshow(I2);



% --- Executes on selection change in drawPLines.
function drawPLines_Callback(hObject, eventdata, handles)
% hObject    handle to drawPLines (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns drawPLines contents as cell array
%        contents{get(hObject,'Value')} returns selected item from drawPLines
handles = guidata(hObject);
str = get(hObject, 'String');
val = get(hObject,'Value');
switch str{val}
    case 'X'
        [x,y]= getVanishingPt('r');
        handles.vX = [x;y;1];
    case 'Y'
        [x,y]= getVanishingPt('g');
        handles.vY = [x;y;1];
    case 'Z'
        [x,y]= getVanishingPt('b');
        handles.vZ = [x;y;1];
end
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function drawPLines_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drawPLines (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in setReference.
function setReference_Callback(hObject, eventdata, handles)
% hObject    handle to setReference (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns setReference contents as cell array
%        contents{get(hObject,'Value')} returns selected item from setReference
global plots;
handles = guidata(hObject);
str = get(hObject, 'String');
val = get(hObject,'Value');
if ~handles.O
    msgbox('Select the Origin first!');
else
    switch str{val}
        case 'X'
            disp('Select the refrence point in X direction');
            [x,y] = ginput(1);
            plots(size(plots,2)+1) = plot(x,y,'*');
            plots(size(plots,2)+1) = plot([x handles.O(1)],[y handles.O(2)]);
            handles.refX = [x;y;1];
            answer = inputdlg('Enter reference length','Reference Length',[1 30],{'0'});
            %insertText(get(gca,'Children'),[(x+handles.O(1))/2 (y+handles.O(2))/2], answer);
            handles.refXlen = str2double(answer);
        case 'Y'
            disp('Select the refrence point in Y direction');
            [x,y] = ginput(1);
            plots(size(plots,2)+1) = plot(x,y,'*');
            plots(size(plots,2)+1) = plot([x handles.O(1)],[y handles.O(2)]);
            handles.refY = [x;y;1];
            answer = inputdlg('Enter reference length','Reference Length',[1 30],{'0'});
            %insertText(get(gca,'Children'),[(x+handles.O(1))/2 (y+handles.O(2))/2], answer);
            handles.refYlen = str2double(answer);
        case 'Z'
            disp('Select the refrence point in Z direction');
            [x,y] = ginput(1);
            plots(size(plots,2)+1) = plot(x,y,'*');
            plots(size(plots,2)+1) = plot([x handles.O(1)],[y handles.O(2)]);
            handles.refZ = [x;y;1];
            answer = inputdlg('Enter reference length','Reference Length',[1 30],{'0'});
            %insertText(get(gca,'Children'),[(x+handles.O(1))/2 (y+handles.O(2))/2], answer);
            handles.refZlen = str2double(answer);
    end
end
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function setReference_CreateFcn(hObject, eventdata, handles)
% hObject    handle to setReference (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in transform.
function transform_Callback(hObject, eventdata, handles)
% hObject    handle to transform (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns transform contents as cell array
%        contents{get(hObject,'Value')} returns selected item from transform
handles = guidata(hObject);
str = get(hObject, 'String');
val = get(hObject,'Value');

if ~handles.P
        msgbox('Click START to do calculation first!');
else
    im = handles.g;
    
    height = size(im, 1);
    width = size(im, 2);

    switch str{val}
        case 'X-Y'
            invH = inv(handles.Hxy);
            disp('X-Y plane');
            transformHelper(invH, im, height, width);
        case 'Y-Z'
            invH = inv(handles.Hyz);
            disp('Y-Z plane');
            transformHelper(invH, im, height, width);
        case 'X-Z'
            invH = inv(handles.Hxz);
            disp('X-Z plane');
            transformHelper(invH, im, height, width);
    end
    
    guidata(hObject, handles);
end


% --- Executes during object creation, after setting all properties.
function transform_CreateFcn(hObject, eventdata, handles)
% hObject    handle to transform (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% % --- Executes on button press in labelCoor.
% function labelCoor_Callback(hObject, eventdata, handles)
% % hObject    handle to labelCoor (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% global plots;
% disp('Input 3d corrdinates of six points')
% if ~exist('M.mat', 'file')
%     M = zeros(12,12);
%     for i = 1:6
%         disp('Click a known point')
%         [u,v] = ginput(1);    
%         plots(size(plots,2)+1) = plot(u,v,'*');
%         answer = inputdlg({'X','Y','Z'},...
%                   '3-D coordinates', [1 30; 1 30; 1 30]);
%         wX = str2double(answer(1)); % world X
%         wY = str2double(answer(2));
%         wZ = str2double(answer(3));
%         M(2*i-1,:) = [wX wY wZ 1 0 0 0 0 -u*wX -u*wY -u*wZ -u];
%         M(2*i,:) = [0 0 0 0 wX wY wZ 1 -v*wX -v*wY -v*wZ -v];
%     end
%     M = M(1:11,:)
%     save('M.mat','M');
% else
%     load('M.mat','M');
%     M
% end
% b = zeros(11,1);
% x = pinv(M)*b


% --- Executes on button press in selectIP.
function selectIP_Callback(hObject, eventdata, handles)
% hObject    handle to selectIP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global plots;
handles.ThreeDpos = zeros(3,0);
b0 = handles.O;
t0 = handles.refZ;
H = handles.refZlen;
horizon = real(cross(handles.vY, handles.vX));
length = sqrt(horizon(1)^2 + horizon(2)^2);
horizon = horizon/length;

while 1
    disp('Click a base point or press q to stop')
    [x1,y1,b] = ginput(1);    
    if b=='q'        
        break;
    end
    plots(size(plots,2)+1) = plot(x1,y1,'*');
    b = [x1;y1;1];
    
    disp('Click an interesting point');
    [x2,y2] = ginput(1);
    plots(size(plots,2)+1) = plot(x2,y2,'*');
    r = [x2;y2;1];
    
    line1 = real(cross(b0, b));
    v = real(cross(line1, horizon));
    v = v/v(3);

    line2 = real(cross(v', t0));
    vertical_line = real(cross(r, b));
    t = real(cross(line2, vertical_line));
    t = t/t(3);
    height = H*sqrt(sumsqr(r-b))*sqrt(sumsqr(handles.vZ'-t))/...
        sqrt(sumsqr(t'-b))/sqrt(sumsqr(handles.vZ-r))
    
%     figure();
%     imagesc(handles.g);
%     hold on;
%     plot([handles.vY(1) handles.vX(1)], [handles.vY(2) handles.vX(2)]);
%     plot([v(1) b0(1)], [v(2) b0(2)], 'r');
%     plot([t0(1) b0(1)], [t0(2) b0(2)], 'r');
%     plot([v(1) t0(1)], [v(2) t0(2)], 'r');
%     plot([v(1) t(1)], [v(2) t(2)], 'g');
%     plot([b(1) t(1)], [b(2) t(2)], 'g');
%     plot([b(1) v(1)], [b(2) v(2)], 'g');
%     plot([b(1) r(1)], [b(2) r(2)], 'b');
%     axis equal;
%     axis image;
    
    answer = inputdlg('Add this point? (Y/N)', 'Adding',[1 30],{'Y'});
    if char(answer) == 'Y'
        Hz = [handles.P(:,1) handles.P(:,2) handles.P(:,3).*height+handles.P(:,4)];
        pos = Hz\r;
        worldCoor = [pos(1)/pos(3); pos(2)/pos(3); height]
        handles.ThreeDpos(:, end+1) = worldCoor;
        
        answer = inputdlg('Continue adding points on the same Z? (Y/N)', 'Same height adding',[1 30],{'Y'});
        if char(answer) == 'Y'
            while 1
                disp('Click interesting points on the same height or press q to stop')
                [x,y,b] = ginput(1);    
                if b=='q'        
                    break;
                end
                plots(size(plots,2)+1) = plot(x,y,'*');
                newPt = [x;y;1];
                Hz = [handles.P(:,1) handles.P(:,2) handles.P(:,3).*height+handles.P(:,4)];
                pos = Hz\newPt;
                worldCoor = [pos(1)/pos(3); pos(2)/pos(3); height]
                handles.ThreeDpos(:, end+1) = worldCoor;
            end
        end
    end

end
guidata(hObject, handles);

% --- Executes on button press in build.
function build_Callback(hObject, eventdata, handles)
% hObject    handle to build (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

pos = [handles.ThreeDpos;1:size(handles.ThreeDpos,2)]

% scatter plot
X = pos(1,:);
Y = pos(2,:);
Z = pos(3,:);
textCell = arrayfun(@(x,y,z) sprintf('(%3.2f, %3.2f, %3.2f)',x,y,z),X,Y,Z,'un',0);
figure;
scatter3(X,Y,Z);
for ii = 1:numel(X) 
    text(X(ii)+.02, Y(ii)+.02, Z(ii)+.02, textCell{ii},'FontSize',8) 
end

valstring = input('Which coordinates to use for current patch? => ', 's'); %input 1,...,N (#coordinates) in order
valparts = regexp(valstring, '[ ,]', 'split');
coord = str2double(valparts);
num_coord = size(coord, 2);

filename = input('texture file name? (with extension) => ', 's');

cur_texture = imread(filename);
cur_height = size(cur_texture, 1);
cur_width = size(cur_texture, 2);
figure, imshow(cur_texture);
disp('Select texture coordinated in order');
texCoor = zeros(2,0);
for i = 1:num_coord
    [x,y] = ginput(1);
    %plot(x,y,'*');
    texCoor(:, end+1) = [x/cur_width; (cur_height-y)/cur_height]
end

if ~exist('test.txt', 'file')
    fid = fopen('test.txt','w');
    fprintf(fid,'#VRML V2.0 utf8\n\nCollision {\n collide FALSE\n children [\n ]\n }');
    fclose(fid);
end
%delete last two lines
fcontent = fileread('test.txt');
fid = fopen('test.txt','w');
fwrite(fid, regexp(fcontent, '.*(?=\n.*?)', 'match', 'once'));
fclose(fid);
fcontent = fileread('test.txt');
fid = fopen('test.txt','w');
fwrite(fid, regexp(fcontent, '.*(?=\n.*?)', 'match', 'once'));
fclose(fid);
%append a new Shape & add back last two lines
fid = fopen('test.txt','a');
fprintf(fid, '\n Shape {\n  appearance Appearance {\n   texture ImageTexture {\n   url "');
fprintf(fid, filename);
fprintf(fid, '"\n  }  \n  }\n   geometry IndexedFaceSet {\n   coord Coordinate {\n   point [\n');
for i = 1:num_coord
    cur = coord(i);
    fprintf(fid, '     %9.5f %9.5f %9.5f, \n', pos(1,cur),pos(2,cur),pos(3,cur));
end
fprintf(fid, '\n   ]\n   }\n   coordIndex [\n    ');
for i = 1:num_coord
    fprintf(fid, '%i,', i-1);
end
fprintf(fid, '%i,', -1);
fprintf(fid, '\n   ]\n   texCoord TextureCoordinate {\n    point [\n');
for i = 1:num_coord
    fprintf(fid, '     %3.2f %3.2f,\n', texCoor(1,i), texCoor(2,i));
end
fprintf(fid, '\n    ]\n   }\n   texCoordIndex [\n    ');
for i = 1:num_coord
    fprintf(fid, '%i,', i-1);
end
fprintf(fid, '%i,', -1);
fprintf(fid, '\n   ]\n   solid FALSE\n  }\n}');
fprintf(fid, '\n ] \n}');
fclose(fid);