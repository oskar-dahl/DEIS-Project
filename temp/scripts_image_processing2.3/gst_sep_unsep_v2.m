function [I20,I11] = gst_sep_unsep(inim,dx,gx,dy,gy,gamma,w2,do_f,th)
%GST_SEP_UNSEP  Computes the generalized structure tensor for a symmetry order.
%
%   The input image must be supplied in INIM, having type double. The
%   symmetry type is given by the parameter TYP.
%
%   The parameter w1 represents the filter used in the first level. It has 2 rows. The first row is the
%   derivative filtering whereas the second row is the integrative
%   filtering.
%
%   The parameter w2 is a 2D complex filter used in the second
%   level.
%
%   The parameter do_f is the sub sampling rate. Default is 2.

%%CPU-Time

 %i20_time=[];
 %tic %1: start measuring the cpu-time
if nargin<8
    do_f=2;
end

    % dxf=filter2(gy,  filter2(dx,inim));  %derivate inim with respect to x
% dyf= filter2(gx, filter2(dy,inim)); %derivate inim wrt  y
dxf= conv2(conv2(inim,dx,'same'),gy,'same');  %derivate inim with respect to x
dyf= conv2(conv2(inim,dy,'same'),gx,'same'); %derivate inim wrt  y
[agrad,mgrad]=cart2pol(dxf,dyf);
%th=sum(gy)*sum(abs(dx))/2;  
mgrad_th=zeros(size(mgrad));
bm=(length(dx)+1)/2; %odd filter size is assumed when computing boundary margins.o
mgrad_th(bm:(end-bm+1),bm:(end-bm+1))=mgrad(bm:(end-bm+1), bm:(end-bm+1));
mgrad_th(mgrad_th<th)=0;

if (gamma ~= 1.0)
    mgrad_th=mgrad_th.^(gamma);
end

dagrad=2*agrad;
[LSX,LSY]=pol2cart(dagrad,mgrad_th);
I20=complex(LSX,LSY);


%%CPU-Time
%i20_time= [i20_time toc], Info=['<-ILS by separable filtering including gamma correction of grad-magnitudes.'] %1: append the cpu-time measurement obtained between the last tic and the current statement


% %Filters:  x^2-y^2 -i*2*x*y
% %generate 2 1-D integrative filters
% xxg=gaussgen(sma2,'xxg',[1,round(sma2*7)]);%generate large horizontal xxg  filter
% yyg=xxg';
%
% xg=gaussgen(sma2,'dxg',[1,round(sma2*7)]);%generate large horizontal xg  filter
% yg=-xg';
%
%
% gx2=gaussgen(sma2,'gau',[1,round(sma2*7)]);%generate large horizontal gaussian filter
% gy2=gx2';%generate vertical gaussian filter
%
%
%
%  i20=filter2(gy2, filter2(xxg,LS)) - filter2(gx2, filter2(yyg,LS)); %average the orientation tensor with large gaussian
%  i20=i20+j*2*filter2(yg, filter2(xg,LS));
%  i11= filter2(gy2, filter2(xxg,ALS))+filter2(gx2, filter2(yyg,ALS)); %average the magnitutude of the orientation tensor with large gaussian

%
% typ=-2; sm=sma2; gammaf=100 ;
% %typ=0; sm=0.33; gammaf=1;
% h2=symdergaussgen(typ,sm,gammaf);


%%CPU-Time
%I20=double(I20);
%w2=double(w2);
%tic %2: start measuring the cpu-time

nnz_w2_log= nnz(w2)>1;
nnz_mgrad_th_log= nnz(mgrad_th)>1;
if (nnz_w2_log)
    if (nnz_mgrad_th_log)
        %%CPU-Time
        %    tic %3: start measuring the cpu-time
        I20=conv2(I20(1:do_f:size(I20,1),1:do_f:size(I20,2)),(do_f)^2*w2(1:do_f:size(w2,1),1:do_f:size(w2,2)),'same');
        %    I20=filter2(w2,I20);
    else
        I20(:)=0;
    end
end


if (1<nargout)
    I11=zeros(imsize(I20));
    if (nnz_w2_log)
        if (nnz_mgrad_th_log)
            I11=conv2(mgrad_th(1:do_f:size(mgrad_th,1),1:do_f:size(mgrad_th,2)),(do_f)^2*abs(w2(1:do_f:size(w2,1),1:do_f:size(w2,2))),'same');
        end
    end
end



%%CPU-Time
% i20_time= [i20_time toc], Info=['<-complex-filtering']  %2: append the cpu-time measurement obtained between the last tic and the current statement
%%CPU-Time
% si20_time=sum(i20_time)
% ri20_time=i20_time/si20_time
end

    