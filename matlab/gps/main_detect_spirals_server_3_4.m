function [pos] = main_detect_spirals_server_3_4(imrgb) 

% version 3_4 compared to 3.3 :
% 1. Number of spirals in (spiral) alphabet is Ns=10.
% 2. thresh is now normalized between 0 and 1. It aims to be applied to the magnitudes of I_20 (of spiral detection)
% in the entire image.The effect of this is not visible in this script but in functions wherein the input image is
%    normalized to 1. In turn this makes IILS image to have a more predictable interval for magnitudes. As it stands it
%    is intended for full image.
% 3. calls spiral_detection_bufv2 (instead of spiral_detection_buf2) for
% spiral detection
% 4. Works best for original images without compensation for lens
% distorsion, as they come from the camera at 600x800 resolution.
% The compensation includes currently a warping which affects local
% orientations too much at high frequencies. Perhaps it would work if the
% original image size would be larger.

% version 3_3 compared to 3.2 (version given to tommy):
%1. sma1=0.6 (0.75) gammaf=60 (15.5) gamma=0.11 (1)
%2. It has better exception handling when zero spirals are present in
%   mark_obj_2.
%3. It detects 78 spirals out of 84 in eighty_four_spirals_16_2016.tif (with zero false-acceptance).
%     If  version 3_2 with  gamma=0.11 (instead of 1) is run on eighty_four_spirals_16_2016.tif,
%     75 spirals are detected (with zero FA). The missed spirals are in the
%     right down corner in both cases (version 3_2_).
%     This tells that the other parameter changes have marginal effect, when gradient contrast is good.

% version 3_3 compared to 3.2.1:
%1. Number of spirals in the alphabet is  7 (3).
%2. sma2=9 (30); and gammaf=60 (6) gamma=0.11 (1)
%
%version 3.2.1:
%1. this is the version given to Klas (albeit called version 3_2 there)
%2. Ns=3; (7),  sma2=30; (9) gammaf= 6  (15.5)

%%The following matlab rectangles are obtained by mrect=getrect()
% A matlab rectangle has the order: x,y,xsize,ysize...
%%CAUTION: There is a bug in getrect. it may return xs,ys that can be too large.
% mrect =[1    16   140   115];
% mrect =[640     2   123   185];
% mrect =[  1   456    87   143];
% mrect =[716   454    84   146];

%imrgb is the full input image
%This is how to read directly from camera as http server.

%1. roi is the full image %If that is not what you want ....comment it
% imsz=[600 800]; %Image size to be pulled out from the
max_pixv=255;   %We  know that the image is u8...
% imsz=[480 640]; %Image size to be pulled out from the
%  mrect =     [1    1   imsz(2) imsz(1)];
% mrect =     [617.0000  449.0000  180.0000  149.0000];

thresh=0.40;%Currently Global  image treshold for I20magnitudes... since the software is used as a gps simulator
%If Roi rectangle is used this can be different...and adapted
%to maximum within each rectangle. this relative tresholding is
%(to be currently) done by adapting
%spiral_detection_bufv2...

gamma=0.20; %This amplifies gradient magnitudes. Applied after the nominal treshold EPS*max_nominal_amplification
%in spiral_detecton_bufv2.
%0.11;%0.35;%0.85;%1 %0.85;


verbose=1;   %Images of lots of intermediery computations are displayed...
verbose2=1;  %Only minimum display: results overlayed...
verbose3=0; %CPU-times...[Absolute; Relative]. No display

%2. roi is a rectangle containing the top pie-robot...
% mrect=[ 1205    121    286    301]; thresh=0.2;  %Notice we now can  lower
%threshold to allow full
%detection even if the spiral
%certainty is low for one
%spiral. This is because
%our Threshold is relative, that is, if the maximum
%of |I20| occuring in the
%roi is I20max, the actual
%threshold used to isolate spiral-center (blobs) is
%thresh*I20max. It  means
%that reducing the size of
%roi helps in that we
%reduce I20max compared to
%having found it from full image as roi, where a higher I20max risk to occur.
%HOWEVER, IF CHANGING THRESHOLD
%DECREASES FALSE ACCEPTANCE
%ERRORS (FA), IT ALWAYS WORSENS (INCREASES) FALSE
%REJECTION (FR) ERRORS. IT IS NOT
%POSSIBLE TO REDUCE FA AND
%FR ERRORS AT THE SAME
%TIME BY CHANGING A
%THRESHOLD. A COMPROMISE
%IS TO TRY TO KEEP IT AS
%HIGH AS POSSIBLE WHILE WE CAN DETECT SPIRALS EVERYWHERE IN THE IMAGE PLANE.
%3. roi is a rectangle containing the bottom pie...
% mrect=[  1253    408    301    250]; thresh=0.6;  %Notice we now can afford HIGHER treshold.


%% Initialization of filters. This must not be done for every image....It is sufficient once.
do_f=2; %down sampling factor compared to the original input image, for spiral detection
Ns=10; %Number of spirals in the alphabet
%Initialization: produce centroid masks (rowmask and colmask)
Ncm=7; Nch=(Ncm-1)/2;
colmask=ones(Ncm,1)*[-Nch:Nch]; % rowmask=reshape(rowmask, [Ncm*Ncm,1]);rowmask=rowmask';
rowmask=[-Nch:Nch]'*ones(1,Ncm); %colmask=reshape(colmask, [Ncm*Ncm,1]);colmask=colmask';
global spiral_obj_ids;
global Picture_Toc;


%initialization: spiral detection
%generate 4 1D derivative filters and a 2D complex filter.
sma1=0.6; %0.75
gx=gaussgen(sma1,'gau');
dx=gaussgen(sma1,'dxg');
gy=gx';
dy=-dx';

% Hpeak=(sma1^-1)*exp(-1/2);
% Hpeak=0.5*Hpeak; %Max amplitude is 0.5 when the image is positive andn in [0 1].
% dy=-dx'/Hpeak;
% gy=gx'/Hpeak;
% L1nrm=sum(abs(dx));
% gx=gx/sqrt(L1nrm);
% dx=2*dx/sqrt(L1nrm);
% gy=gy/sqrt(L1nrm);
% dy=2*dy/sqrt(L1nrm);


%                %The second 2 is for the fact that a sinusoid can at most have half of the
%                %input pixel range as amplitude. If the input pixel range is 1 then
%                %the output pixels will bi sinusoids amplitudes between 0
%                %and 0.5 at most.


sma2=9; %The division 2 is there because we will use the subsampled gradient image when applying hs...
sma=[sma1,sma2];
typ=2; %convolution is utilized...It means that there is already a conjugation in the filtering
sm=double(-sma2); gammaf=60;%15.5;
% ...if sm is negative then symdergaussgen interprets it as the radius
%   that we wish that the max filter values will occur (instead of standard
%   deviation.
% hs=symdergaussgen(typ,sm,gammaf);
sizev=[-1,-1];
if verbose
    verb=1140;
else
    verb=0;
end
normalize=1;
[hs]=symdergaussgen3(typ,sm,gammaf,sizev,verb,normalize);
%hs=hs.*abs(hs);
%            [i,j,s] = find(hs);
%                [m,n] = size(hs);
%                hs = sparse(i,j,s,m,n);
scaling=['sclon'];

sma3=4*sma2;
gx3=gaussgen(sma3,'gau');
gy3=gx3';



%% Read image
if verbose3
    ptime=[]; tic;
end

%imrgb=imread('images/distorted800x600.jpg');
%imrgb=imread(['http://ideax3.hh.se/axis-cgi/jpg/image.cgi?resolution=' num2str(imsz(2)) 'x' num2str(imsz(1))]);
%  imrgb=imread('images\undistorted800x600.jpg');

% imrgb=imread('images\my_image_feed_800-600-undistorted.jpg');
%  imrgb=imread('images\my_image_feed_1280-960.jpg');
% imrgb=imread('images\my_image_feed-640-480-undist.jpg');
%imrgb=imread('images\my_image_feed-640-480.jpg');
imsz=size(imrgb);%[480 640]; %Image size to be pulled out from the
mrect =     [1    1   imsz(2) imsz(1)];
%

if 0%(verbose2 ||  verbose)
    ifh=100;
    %figure(ifh);imshow(imrgb,[0 255],'Border','tight','InitialMagnification',100);
    cp=get(ifh, 'position'); cp(1:2)=[276   118];
    set(ifh, 'position',cp);
    imwrite(imrgb, 'org_to_be_sent_to_jb.tiff');
end
if verbose
    %figure(3);imshow(imrgb,'Border','tight','InitialMagnification',100);
    %figure(33);imshow(imresize(imrgb,0.5),'Border','tight','InitialMagnification',100);
end
imrgb(1:15,1:100,:)=imrgb(1:15,(end-100+1):end,:);  %Put "zero" ...deleting bright clock-display to reduce interference.

% %
% imrgb=imread('http://ideax3.hh.se/axis-cgi/jpg/image.cgi?resolution=1600x1200');
% %old


%Region Of Interest, roi, is in the order: rowTL,colTL,rowBR, colBR
%Translate matlab rectangle to roi
roi=mrect;
botr=roi(1:2)+roi(3:4)-1;
roi=[roi(1:2),botr];
roi=[roi(2),roi(1),roi(4),roi(3)];
roi_imrgb=imrgb(roi(1):roi(3),roi(2):roi(4),:);
roi_im=sum(imrgb(roi(1):roi(3),roi(2):roi(4),:),3)/3/max_pixv;
%max_pixv division is for normalization
EPS=0.01;
roi_im_dark_pixs= (roi_im+1<(EPS*10));
too_dark= (0.5 < mean(roi_im_dark_pixs(:))); %Dark pixels cover more than half of the total image area

if ~too_dark
    if verbose
        %Info=['....ENTERING ROI COMPUTATIONS....ENTERING ROI COMPUTATIONS....ENTERING ROI COMPUTATIONS...']
    end
    if verbose
        ifh=101;
        %figure(ifh);imshow(roi_im,[0 1],'Border','tight','InitialMagnification',100);
    end
    %     gxlight=gaussgen(40,'gau');
    %     gylight=gxlight';
    
    
    
    % loc_mean=conv2(gylight,gxlight,roi_im,'same');
    if verbose3
        ptime=[ptime toc];tic;
    end
    %     loc_mean=imfilter(imfilter(roi_im, gylight, 'replicate'), gxlight, 'replicate');
    
    
    %     C:\Program Files\MATLAB\R2015a\toolbox\images\images\private\getPaddingIndices.m  % Private to images
    
    %     aIdx = getPaddingIndices(size(roi_im),([length(gxlight) length(gxlight)]-1)/2, 'replicate', 'both');
    %     b = roi_im(aIdx{:});
    % hej=imresize(imresize(b,0.008,'bicubic'),size(b),'bicubic');
    
    loc_mean=imresize(imresize(roi_im,0.008),size(roi_im));
    roi_im=roi_im./loc_mean;
    roi_im=roi_im/max(roi_im(:));
    roi_im=roi_im*2-1; %normalize between -1 and 1.
    if verbose
        %figure(105);imshow(loc_mean,[0 1],'Border','tight','InitialMagnification',100);
        %figure(113);imshow(roi_im,[-1 1],'Border','tight','InitialMagnification',100);
    end
    if verbose3
        ptime=[ptime toc];tic;
    end

     [spiral_obj_roi,I20nmxs,I20]=spiral_detection_buf_v2(roi_im,sma,scaling,gamma,dx,gx,dy,gy,hs,rowmask,colmask,thresh,do_f);
    if verbose
        %figure(4);I20rgb=lsdisp(I20,1,'scloff'); %truesize; axis off
        %figure(6);imshow(I20nmxs,'Border','tight','InitialMagnification',100);
        %figure(7);lsdisp(I20nmxs.*exp(i*angle(I20)),1,'scloff');
        %         dcm=datacursormode(gcf);
        %         set(dcm,'UpdateFcn', [])
    end
    
    spiral_obj=[];
    if size(spiral_obj_roi,1)>0
        spiral_obj=[roi(1)+spiral_obj_roi(:,1)-1,roi(2)+spiral_obj_roi(:,2)-1,spiral_obj_roi(:,3)];
    end
    if verbose3
        ptime=[ptime toc];
    end
    if size(spiral_obj_roi,1)>0
        % ids=(Ns-1)/2-round((pi-mod(spiral_obj(:,3),2*pi))/(2*pi/Ns)); %this does angle tresholding, id mapping,..etc
        ids=round((mod(spiral_obj(:,3),2*pi)-2*pi/Ns/2)/(2*pi/Ns)); %...Same. Just question of programming taste...
    else
        ids=[];
    end
    Number_of_spiral_obj=size(ids,1);
    
    if size(spiral_obj_roi,1)>0
        spiral_obj_rc_do_f=spiral_obj;
        spiral_obj_rc_do_f(:,1:2)=spiral_obj(:,1:2)/do_f;
        
        ind_cert=sub2ind(size(I20nmxs), round(spiral_obj_rc_do_f(:,1)),round(spiral_obj_rc_do_f(:,2)));
        cert=abs(I20(ind_cert));
        
        if verbose==2
            %Debugging/info printout...
            spiral_obj_rc_do_f_ids_sorted_=sortrows([spiral_obj_rc_do_f, ids,cert],[4,5])
        end
        
        spiral_obj_sorted=sortrows([spiral_obj, ids,cert],[4,5]);
        [~,ia]=unique(spiral_obj_sorted(:,4),'last'); %to break identity-tie, keep id with highest certainty
        spiral_obj_ids_sorted_unique=spiral_obj_sorted(ia,:);
        if verbose==2
            spiral_obj_ids_sorted_unique
        end
        ids=spiral_obj_ids_sorted_unique(:,4);
        spiral_obj_ids=-ones(Ns,5);
        Ins= (-1<spiral_obj_ids_sorted_unique(:,4)) & (spiral_obj_ids_sorted_unique(:,4)<Ns) ;
        
        spiral_obj_ids(ids(Ins)+1,:)=spiral_obj_ids_sorted_unique(Ins,:);
        %spiral_obj_ids(ids(Ins)+1,:)=spiral_obj_ids_(Ins,:);
        
    if (verbose2 ||  verbose)
        %display(['Down sampling factor to detect spirals...do_f=' num2str(do_f)]);
        %display(['Number of spirals, including doublettes....=' num2str(Number_of_spiral_obj)]);
        %spiral_obj_ids
        
        spiral_obj_ids_group5 = spiral_obj_ids;
        spiral_obj_ids_group5(:, 1) = spiral_obj_ids(:, 2);
        spiral_obj_ids_group5(:, 2) = spiral_obj_ids(:, 1);
        
        %figure(999);
        %scatter(spiral_obj_ids_group5(:,1), spiral_obj_ids_group5(:,2))
        
       
        %imrgbmark= imrgb(1:2:end,1:2:end,:);
        %nsc=4; %cross half size
        %pv=0;  %pixel value of the cross
        %imrgbmark=mark_obj_3(imrgbmark,spiral_obj_ids_sorted_unique(Ins,:), nsc, pv,do_f);
        %         %figure(5);imshow(imrgbmark,'Border','tight','InitialMagnification',100);
        %figure(55);imshow(imrgbmark,'Border','tight','InitialMagnification',100);
        %pv=1;  %pixel value of the cross
        %loc_im=mark_obj_2(zeros(size(I20)),spiral_obj_ids_sorted_unique(Ins,:), nsc, pv,do_f);
        %figure(555);imshow(loc_im,'Border','tight','InitialMagnification',100);
        %loc_im=mark_obj_2(zeros(size(I20)),spiral_obj_ids_sorted_unique(Ins,:), nsc, pv,do_f);
        %figure(5);imshow(loc_im,'Border','tight','InitialMagnification',100);
        pos = spiral_obj_ids_group5;
        return
    
    end
    end
    
    
    %    imwrite(imrgb, 'images/org_plus_cross.tiff');
    if verbose3
        display('Columns (Sec)')
        display('1: Image reading.')
        display('2: Smoothing by resizing.')
        display('3: Spiral detection.')
        display('4: Total.')
        ptime=[ptime  sum(ptime)];
        ptime=[ptime;
            ptime/ptime(end)]
    end
else
    display('Image is too dark to recognize spirals....')
    %figure(1);imshow(imrgb,[0 1],'Border','tight','InitialMagnification',100);
end

%%End: Info
aa=0;