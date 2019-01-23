%manual version control
%1. spiral_detection_v2.m is obtained from spirala_detection_buf.m
%Main differences.
%2. inim is assumed to be normalized between -1 and 1, double float
%the thres is between 0 and 1 and applied to the output of abs(I20).
%3. centrv2 is called instead of centr. The former does not use thresh as
%its input is already tresholded.

function  [o,I20nmxs,I20,I11]=spiral_detection_buf(inim,std,scaling,gamma,dx,gx,dy,gy,h2,rowmask,colmask,thresh,do_f)
% (C) Josef Bigun
%Computes the linear symmetry (logzLS) in local neighborhoods in log(z) coordinates.
%The details of the theory is given in
% @Article{bigun97cviu2,
%   author =       {J. Bigun},
%   title =        {Pattern
% recognition in images by symmetries and coordinate transformations},
%   journal =      cviu,
%   year =         {1997},
%   OPTkey =       {},
%   volume =       {68},
%   number =    {3},
%   OPTmonth =     {},
%   pages =     {290-307},
%   OPTannote =    { %old key: bigun96cviu2},
% OPTnote =      {\htmladdnormallink{bigun97cviu2.pdf}{bigun97cviu2.pdf}},
% keywords =     "pattern recognition theory; local symmetry; lie
%                  groups; infinitesimal operators; image analysis;
%                  computer vision; matching; local orientation tensors;
%                  infinitesimal linear symmetry",
% }
%The function spiral_detection can be called with 1, 2, 3, or 4  input arguments as:
%spiral_detection(inim), spiral_detection(inim,std), spiral_detection(inim,std,scaling), spiral_detection(inim,std,scaling,gamma).
% The default values of the omitted arguments are
%      std=[0.8, 3.5];
%      scaling='sclon';
%      gamma=0.8;
% gamma is used to exponentiate the gradient magnitudes.
% If std(1)<0.9 then the size of the original image is doubled.
%See also lsdisp to display LS.


% if (1 < nargin)
sma1=std(1); sma2=std(2);

% %generate 4 1-D derivative filters.
% dx=gaussgen(sma1,'dxg',[1,round(sma1*6)]);
% gx=gaussgen(sma1,'gau',[1,round(sma1*6)]);
% dy=-dx';
% gy=gx';
% 
%typ=2; sm=sma2; gammaf=100 ;
% %typ=0; sm=0.33; gammaf=1;
% h2=symdergaussgen(typ,sm,gammaf);
spiral_time=[];
% if 3<nargout
%     [I20,I11] = gst_sep_unsep(inim,dx,gx,dy,gy,gamma,h2);
% else
    tic %2.1: start measuring the cpu-time
    max_nominal_amplification=sma1^(-1)*exp(-0.5);
    EPS=0.01;
    th_grad=EPS*max_nominal_amplification; %below this magnitude gradients will be zeroed...
    [I20] = gst_sep_unsep_v2(inim,dx,gx,dy,gy,gamma,h2,do_f,th_grad);
% whos inim
    %   spiral_time= [spiral_time toc], Info=['<-I20 by non-separable filtering'] %2.1: append the cpu-time measurement obtained between the last tic and the current statement
% end

%put zeros around the boundary of I20
%compute the absolute values of pixels in I20
%normalize the absolute values with maximum absolute value
%gamma correct them with value 3.5
%Keep only those which are half as much as the maximum.
tic %2.2: start measuring the cpu-time
%b=round(3*sma1)+round((size(h2,1)-1)/2);
b=round(3*sma1)+round((size(h2,1)/2-1)/2);

I20(1:b-1,:)=0; 
hs=size(I20,1);
I20(hs-b:hs,:)=0;
I20(:,1:b-1)=0;
ws=size(I20,2);
I20(:,ws-b:ws)=0;
%effective rectangle is (b:hs-b-1, b:ws-b-1)
mI20=zeros(size(I20));
mI20(b:hs-b-1, b:ws-b-1)= abs(I20(b:hs-b-1, b:ws-b-1));
loc_msmall=(mI20<thresh); %mI20 is compared to "absolute" treshold
loc_mlarge=  ~loc_msmall;

%I20(loc_msmall)=0; %I20 remains untresholded to avoid aliasing after
%centroid computation
mI20(loc_msmall)=0;
mxm=max(max(mI20(loc_mlarge)));
mI20(loc_mlarge)=mI20(loc_mlarge)/mxm;

%Use either Non-maximum suppression.
tic %2.3: start measuring the cpu-time
% mx_mI20=imdilate(imdilate(byte_mI20,ones(1,3*b)), ones(3*b,1));
% I20nmxs=(mx_mI20 == byte_mI20 ) & logical(byte_mI20);
% %I20nmxs= (mx_mI20 == logical(mI20)) & logical(mI20);
% [ro,co]=find(I20nmxs);
% ao=angle(I20(sub2ind(size(I20),ro,co)));
% o=[ro,co,ao];
% 

%Orcentroids computation to obtain the positions of peaks
[centroids ] = centrv2(mI20,loc_mlarge,rowmask,colmask );
if ~isempty(centroids)
ao=angle(I20(sub2ind(size(I20),round(centroids(:,1)),round(centroids(:,2)))));
centroids=centroids*do_f;
o=[centroids(:,1),centroids(:,2),ao];
else
    o=[];
end
I20nmxs=mI20;
%spiral_time= [spiral_time toc], Info=['<-Centroid'] %2.3: append the cpu-time measurement obtained between the last tic and the current statement

%byte_mI20=uint8(255*mI20);
%figure(10); imshow(byte_mI20);
%I20=exp(i*angle(I20)).*mI20;
%I20nmxs=mx_mI20;

%sspiral_time=sum(spiral_time), Info=['<-Spiral detection total']
%rspiral_time=spiral_time/sspiral_time

% if 3<nargout
%     I11=filter2(abs(h2), abs(LS));
%     I11(1:b-1,:)=0;
%     I11(hs-b:hs,:)=0;
%     I11(:,1:b-1)=0;
%     I11(:,ws-b:ws)=0;
% end

end







