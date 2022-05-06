function varargout = toneCure(varargin)

%% author ��chensc
%% data��  2021/04/05
% TONECURE MATLAB code for toneCure.fig
%      TONECURE, by itself, creates a new TONECURE or raises the existing
%      singleton*.
%
%      H = TONECURE returns the handle to a new TONECURE or the handle to
%      the existing singleton*.
%
%      TONECURE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TONECURE.M with the given input arguments.
%
%      TONECURE('Property','Value',...) creates a new TONECURE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before toneCure_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to toneCure_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help toneCure

% Last Modified by GUIDE v2.5 07-Jun-2021 17:29:29

% Begin initialization code - DO NOT EDIT

cur = cd;
addpath(genpath(cur));
warning off;


gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @toneCure_OpeningFcn, ...
                   'gui_OutputFcn',  @toneCure_OutputFcn, ...
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


% --- Executes just before toneCure is made visible.
function toneCure_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to toneCure (see VARARGIN)

% Choose default command line output for toneCure
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

cla(handles.axes2,'reset');      % ������ն�̬axes1������ 
cla(handles.axes3,'reset');      % ������ն�̬axes1������ 
cla(handles.axes4,'reset');      % ������ն�̬axes1������ 

clear global points;          %%ÿ���������ȫ�ֱ���
global points;                   % �Զ�������
clear global Num;
global  Num;
Num = 1;

points(1,1:2)=0.0;
points(2,1:2)=0.2;
points(3,1:2)=0.4;
points(4,1:2)=0.6;
points(5,1:2)=0.8;
points(6,1:2)=1;

% axes(handles.axes2);

%%    y =  P(1)*X^N + P(2)*X^(N-1) +...+ P(N)*X + P(N+1);
%%   p�ǲ����� N��ָ��λ��

p1 =  polyfit(points(:,1),  points(:,2), 5);       %% y  = x  +  b;
x=0:0.01:1;
y = polyval(p1, x);  

% hold on;
y2 = x ;
plot(x, y2, 'k-.', 'LineWidth', 1, 'parent', handles.axes2);
hold on;
plot(x, y, 'r-', points(:,1), points(:,2), 'b*',  'LineWidth', 1.25, 'parent', handles.axes2) ;
legend('y = x', 'y = f(x)', 'Location', 'northwest');

grid on;
global idx
idx=0;
guidata(hObject, handles);

 


% UIWAIT makes toneCure wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = toneCure_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1

%% ��Բ�ͬͨ��������ȡ���������߹���

clear global Num;
global  Num;
switch(get(handles.popupmenu1,'Value'))
    case 1 
        Num =1;
    case 2
        Num =2;
    case 3
        Num =3;
    case 4
        Num =4;
    case 5
        Num = 5;
     case 6
        Num =6;
    case 7
        Num =7;
    case 8
        Num =8;
    case 9
        Num = 9;
    case 10
        Num =10;
    case 11
        Num =11;
    case 12
        Num =12;

end
guidata(hObject,handles );







% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu4

%% ֱ��ͼ��ʾѡ��
global DealIMG;
global ImgOri;

if (get(handles.popupmenu4,'Value') > 0)
    switch(get(handles.popupmenu4,'Value'))
        case 1
            cla(handles.axes3,'reset');
            axes(handles.axes3);
            histogram(ImgOri);
            hold on;
            histogram(DealIMG);
            grid on
            legend('ԭͼ', '����ͼ')
        case 2
            cla(handles.axes3,'reset');
            axes(handles.axes3);
            histogram(ImgOri);
            grid on
            legend('ԭͼ');
        case  3
            cla(handles.axes3,'reset');
            axes(handles.axes3);
            histogram(DealIMG);
            grid on
            legend('����ͼ');
    end
end
guidata(hObject,handles );










% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% ��ͼ��

guidata(hObject, handles);    %��������
cla(handles.axes2,'reset');      % ������ն�̬axes2������ 
cla(handles.axes3,'reset');
cla(handles.axes4,'reset');
set(handles.popupmenu1,'Value', 1) ;
set(handles.popupmenu4,'Value', 1) ;

clear global  Str;
clear global points;          %%ÿ���������ȫ�ֱ���
clear global Num;

global Str;
global ImgOri;
global  Num;
global points;                   % �Զ�������
global idx

idx=0;
Num = 1;

points(1,1:2)=0.0;
points(2,1:2)=0.2;
points(3,1:2)=0.4;
points(4,1:2)=0.6;
points(5,1:2)=0.8;
points(6,1:2)=1;

% axes(handles.axes2);

%%    y =  P(1)*X^N + P(2)*X^(N-1) +...+ P(N)*X + P(N+1);
%%   p�ǲ����� N��ָ��λ��

p1 =  polyfit(points(:,1),  points(:,2), 5);       %% y  = x  +  b;
x=0:0.01:1;
y = polyval(p1, x);  

% hold on;
y2 = x ;
plot(x, y2, 'k-.', 'LineWidth', 1, 'parent', handles.axes2);
hold on;
plot(x, y, 'r-', points(:,1), points(:,2), 'b*',  'LineWidth', 1.25, 'parent', handles.axes2) ;
legend('y = x', 'y = f(x)', 'Location', 'northwest');
grid on;

[filename, pathname] = uigetfile({'*.*';'*.jpg';'*.png'},'Choose Image');

String = strcat(pathname, filename);
set(handles.edit2, 'string', String);
Str = pathname;

Img=imread([pathname filename]);
axes(handles.axes4);
IMG = [Img;Img];
imshow(IMG);

gray = rgb2gray(Img);
luma = mean(gray(:));

filename = strcat('ԭͼ--', num2str(luma), '--����ͼ--', num2str(luma));
title(filename);

axes(handles.axes3);
histogram(Img);
hold on;
histogram(Img);
grid on;
legend('ԭͼ', '����ͼ');

ImgOri=Img;
handles.ImgOri=Img;

set(gcbf,'WindowButtonDownFcn',@MouseDown);
guidata(hObject, handles);


% --- Executes on mouse motion over figure - except title and menu.
function figure1_WindowButtonMotionFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function MouseDown(~,~)
set(gcbf,'WindowButtonMotionFcn',@MouseMoving);
set(gcbf,'WindowButtonUpFcn',@MouseUp);

function MouseUp(~,~)
set(gcbf,'WindowButtonMotionFcn','');
set(gcbf,'WindowButtonUpFcn','');

clear gloabl DealIMG;

global idx
global DealIMG;
idx=0;
global  Num;
global ImgOri
global myPlot

handles=guidata(gcbo);
[~,~,c]=size(ImgOri);
ImgRes=double(ImgOri)./255;
offset = 128;

y0   = double(ImgOri);
y= 0.299*y0(:,:,1) + 0.587*y0(:,:,2) + 0.114 *y0(:,:,3);
u= - 0.1687*y0(:,:,1) - 0.3313*y0(:,:,2) + 0.5 *y0(:,:,3) + offset;
v = 0.5*y0(:,:,1) - 0.4187*y0(:,:,2) - 0.0813 *y0(:,:,3) + offset;

y  = double(y)/255;
u  = double(u)/255;
v  = double(v)/255;


switch (Num)
    case 1         %default
        for cc=1:c
            ImgRes(:,:,cc)=polyval(myPlot, ImgRes(:,:,cc));
        end
        ImgRes=uint8(ImgRes.*255);
    case  2        %R
        ImgRes(:,:,1)=polyval(myPlot, ImgRes(:,:,1));
        ImgRes=uint8(ImgRes.*255);
        
    case  3        %G
        ImgRes(:,:,2)=polyval(myPlot, ImgRes(:,:,2));
        ImgRes=uint8(ImgRes.*255);
        
    case  4        %B
        ImgRes(:,:,3)=polyval(myPlot, ImgRes(:,:,3));
        ImgRes=uint8(ImgRes.*255);
    case 5          %Y
        y=polyval(myPlot, y);
        y  = double(y)*255;
        u  = double(u)*255;
        v  = double(v)*255;
        R = y + 1.402* (v-offset);
        G = y - 0.34414* (u-offset) - 0.71414* (v-offset);
        B = y + 1.772 *(u-offset);
        ImgRes = cat(3, R, G, B);
        ImgRes=uint8(ImgRes);
    case 6          %U
        u=polyval(myPlot, u);
        y  = double(y)*255;
        u  = double(u)*255;
        v  = double(v)*255;
        R = y + 1.402* (v-offset);
        G = y - 0.34414* (u-offset) - 0.71414* (v-offset);
        B = y + 1.772 *(u-offset);
        ImgRes = cat(3, R, G, B);
        ImgRes=uint8(ImgRes);        
        
    case 7          %V
        v=polyval(myPlot, v);
        y  = double(y)*255;
        u  = double(u)*255;
        v  = double(v)*255;
        R = y + 1.402* (v-offset);
        G = y - 0.34414* (u-offset) - 0.71414* (v-offset);
        B = y + 1.772 *(u-offset);
        ImgRes = cat(3, R, G, B);
        ImgRes=uint8(ImgRes);       
    case 8          %RGB
        for cc=1:c
            ImgRes(:,:,cc)=polyval(myPlot, ImgRes(:,:,cc));
        end
           ImgRes=uint8(ImgRes * 255);         
    case 9             %Lab
            lab = rgb2lab(ImgRes);
            l =  lab(:,:,1)/100;
            el=polyval(myPlot, l)*100;
            lab(:,:,1) = el;
            IMT = lab2rgb(lab);
            ImgRes=uint8(IMT*255);
    case 10
        HSV = rgb2hsv(ImgRes);
        S = HSV(:,:,2);
        HSV(:,:,2)=polyval(myPlot, S);
        ImgRes = uint8(hsv2rgb(HSV)*255);
    case 11
         HSV = rgb2hsv(ImgRes);
        V = HSV(:,:,3);
        HSV(:,:,3)=polyval(myPlot, V);
        ImgRes = uint8(hsv2rgb(HSV)*255);
    case 12
                %% ======%YUV======  %%
        y=polyval(myPlot, y);
        u=polyval(myPlot, u);
        v=polyval(myPlot, v);
        y  = double(y)*255;
        u  = double(u)*255;
        v  = double(v)*255;
        R = y + 1.402* (v-offset);
        G = y - 0.34414* (u-offset) - 0.71414* (v-offset);
        B = y + 1.772 *(u-offset);
        ImgRes = cat(3, R, G, B);
        ImgRes=uint8(ImgRes);   

end


axes(handles.axes4);
DealIMG = ImgRes;

IMG = [uint8(ImgOri); ImgRes];
imshow(IMG)

gray1 = rgb2gray(ImgOri);
luma1 = mean(gray1(:));

gray2 = rgb2gray(ImgRes);
luma2 = mean(gray2(:));

filename = strcat('ԭͼ--', num2str(luma1), '--����ͼ--', num2str(luma2));
title(filename);

axes(handles.axes3);

%% ��ʾֱ��ͼ
histogram(ImgOri);
hold on;
histogram(ImgRes);
grid on
legend('ԭͼ', '����ͼ')



function MouseMoving(~,~)

global points;
global idx;
global myPlot;

handles=guidata(gcbo);
curtP=get(gca,'CurrentPoint');
curtP(curtP>1)=1;
curtP(curtP<0)=0;
x=curtP(1,1);
y=curtP(1,2);

if(idx==0)
    dis=(points(:,1)-x).^2+(points(:,2)-y).^2;
    idx=find(dis==min(dis));
end
points(idx,1)=x;
points(idx,2)=y;
p1 =  polyfit(points(:,1), points(:,2), 5);
% p1 = vpa(p1, 6);        %%����ȡֵ����

formal = strcat('f(x)  =  ', num2str(p1(1)), '*x^5');
t = length(p1) ;
for i = 2:t
    if  i == t
        if (p1(i) > 0)
            formal = strcat(formal, ' +   ', '  ',num2str(p1(i)));
        else
            formal = strcat(formal, num2str(p1(i)));
        end
        
    else
        if (p1(i) > 0)
            formal = strcat(formal, ' +  ','  ', num2str(p1(i)), '*x^', num2str(t-i));
        else
            formal = strcat(formal, num2str(p1(i)), '*x^', num2str(t-i));
        end
    end
    
end

set(handles.edit1, 'string', formal);
cla(handles.axes2,'reset');      % ������ն�̬axes1������ 
cla(handles.axes3,'reset');      % ������ն�̬axes1������ 

myPlot=p1;

x=0:0.05:1;
y = polyval(p1, x); 
y(y>1)=1;
y(y<0)=0;
axes(handles.axes2);

y2 = x ;
plot(x, y2, 'k-.', 'LineWidth', 1, 'parent', handles.axes2);
hold on;
plot(x, y, 'r-', points(:,1), points(:,2), 'b*',  'LineWidth', 1.25, 'parent', handles.axes2) ;
legend('ԭͼ', '����ͼ', 'Location', 'northwest');
grid on;



% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonUpFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)





function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global DealIMG;
global  Str;

if (get(handles.pushbutton3,'Value')>0)
    if isempty(DealIMG)
        errordlg('�����ݣ�����');
    else
        imwrite(uint8(DealIMG), 'deal_IM.bmp');
    end
end
showString = strcat(Str, 'deal_IM.bmp');
set(handles.edit3, 'string', showString);
guidata(hObject, handles);    %��������



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
guidata(hObject, handles);
clear global points;        %%ÿ���������ȫ�ֱ���
clear global idx;
global points;                   % �Զ������� 
global idx;
global ImgOri;
clear global Num;
global  Num;
Num = 1;

if (get(handles.pushbutton5, 'Value') > 0)
    
    cla(handles.axes2,'reset');      % ������ն�̬axes1������
    cla(handles.axes3,'reset'); 
    cla(handles.axes4,'reset'); 
    set(handles.edit1, 'string', ' ');

    points(1,1:2)=0.0;
    points(2,1:2)=0.2;
    points(3,1:2)=0.4;
    points(4,1:2)=0.6;
    points(5,1:2)=0.8;
    points(6,1:2)=1;
    
    p1 =  polyfit(points(:,1),  points(:,2), 5);       %% y  = x  +  b;
    x=0:0.01:1;
    y = polyval(p1, x);
    
    % hold on;
    y2 = x ;
    plot(x, y2, 'k-.', 'LineWidth', 1, 'parent', handles.axes2);
    hold on;
    plot(x, y, 'r-', points(:,1), points(:,2), 'b*',  'LineWidth', 1.25, 'parent', handles.axes2) ;
    legend('y = x', 'y = f(x)',  'Location', 'northwest');
  
    grid on;
    idx=0;
    axes(handles.axes4);
    IMG = [ImgOri; ImgOri];
    imshow(IMG);
    
    gray1 = rgb2gray(ImgOri);
    luma1 = mean(gray1(:));
    
    gray2 = rgb2gray(ImgOri);
    luma2 = mean(gray2(:));
    
    filename = strcat('ԭͼ--', num2str(luma1), '--����ͼ--', num2str(luma2));
    title(filename);
    
    axes(handles.axes3);
    histogram(ImgOri);
    hold on;
    histogram(ImgOri);
    grid on;
    legend('ԭͼ', '����ͼ');
%     set(gcbf,'WindowButtonDownFcn',@MouseDown);
end
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function uipanel4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uipanel4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% I = imread('');




% --- Executes when uipanel4 is resized.
function uipanel4_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to uipanel4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes3


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes2


% --- Executes during object creation, after setting all properties.
function axes4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes4


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clear global points;
clear global idx;
clear global ImgOri;
clear global myPlot;
clear global Num;
global  Num;
Num = 1;

global points;
global idx;

if (get(handles.pushbutton6, 'Value') >0 )

    cla(handles.axes2,'reset');
    cla(handles.axes3,'reset');      % ������ն�̬axes1������
    cla(handles.axes4,'reset');      % ������ն�̬axes1������
    
    set(handles.edit1,'string','');
    set(handles.edit2,'string','');     % ���fram
    set(handles.edit3,'string','');     % ���fram
    
    set(handles.popupmenu1,'Value',1);  %ѡ��˵�
    set(handles.popupmenu4,'Value',1);
    
    points(1,1:2)=0.0;
    points(2,1:2)=0.2;
    points(3,1:2)=0.4;
    points(4,1:2)=0.6;
    points(5,1:2)=0.8;
    points(6,1:2)=1;
    
    p1 =  polyfit(points(:,1),  points(:,2), 5);       %% y  = x  +  b;
    x=0:0.01:1;
    y = polyval(p1, x);
    
    % hold on;
    y2 = x ;
    plot(x, y2, 'k-.', 'LineWidth', 1, 'parent', handles.axes2);
    hold on;
    plot(x, y, 'r-', points(:,1), points(:,2), 'b*',  'LineWidth', 1.25, 'parent', handles.axes2) ;
    legend('y = x', 'y = f(x)',  'Location', 'northwest');
    
    grid on;
    idx=0;
end

% Update handles structure
 guidata(hObject, handles);
