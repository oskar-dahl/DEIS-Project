function [h, x, y]=gaussgen(sma,type,sizev,verbose)
% $Id: gaussgen.m,v 1.7 2013/03/05 16:45:00 josef Exp josef $
% (C) Josef Bigun
% This function generates a 2-D gaussian kernel with standard deviation of sma if type ='gauss'.
% it generates a 2-D  gaussian derivative kernel with standard deviation of sma
% if type='dxg'.  The argument sizev is a vector telling the size of the filter [height,width].
% By using height=1 or widht=1, 1-D gaussian or derivative of gaussians can be generated.
%
% The function can be called with 0, 1, 2, and 3 arguments as:
%   gaussgen, gaussgen(sma), gausssgen(sma,type), and
%   gaussgen(sma,type,sizev)
% The default values of the omitted variables  are
% sma =1;
% type='gau';
% sizev =[1,2*round(pi*std)+1];
% If sma is negative then it is interpreted as reduction factor, not as standard deviation.
% The corresponding sigma is calculated using the max-norm approximation of octave
% reduction. The used sigma is typed.
% If an element of sizev is negative, then that dimension is determined
% automatically as round(6*std).

sm =1;
if nargin>0 sm=sma;  end
if nargin<4 verbose = -1;end

%%Reinterpretation of sm depending on its value
if  ((0<=sm) && (sm<=0.33))
    sizev=[1,1];
    if verbose==1
    Info=['Sigma, if positive, and less than 0.33 it is set to 0.33 as done now.!']
    Info=['...Filter size is set to 1x1!']
    end
    sm=0.33; sizev=[1,1]
elseif (-1<= sm &&(sm<0))
    if verbose==1
    Info=['Illegal sm values!']
    Info=['Reduction factor requests must not be in [-1, 0 )']
    Info=['...Reduction filters must reduce image size with a factor larger than 1 to be meaningful.']
    end
    return
    %If sm is less then -1 then, it is interpreted as
    % reduction-factor for a reducton filter. The sigma for the
    %corresponding filter is computed and printed%
elseif sm<-1
    if verbose==1
    %Info =['... The reduction factor you requested is rf=',num2str(abs(sm)),'.']
    %Info =['The corresponding sm and filter size yields:']
    end
    sm=0.75*(abs(sm)/2)
end

typ='gau';

if nargin>1
    if size(type,2)==2
        if (type=='xg')
            typ='dxg';
        elseif type=='ga'
            typ='gau';
        else
            typ='unk';
        end
    elseif size(type,2)==3
        if ((type=='gau')|(type=='dxg')|(type=='xxg'|(type=='dxx')))
            typ=type;
        else
            typ='unk';
        end
    end
    if typ=='unk'
if verbose==1        
        Info='Valid filter types are gau,{xg|dxg},xxg'  
    end
    return
    end
end

siz =[1,-1];
if (typ == 'dyg')
    siz =[-1,1];
end

if nargin>2
    siz=sizev;
end

if siz(1)<0 siz(1)=1+2*round(pi*sm); end
if siz(2)<0 siz(2)=1+2*round(pi*sm); end


[x,y]=meshgrid(-(siz(2)-1)/2:(siz(2)-1)/2,-(siz(1)-1)/2:(siz(1)-1)/2);

dim=size(x)>1;
CN=(2*pi*sm*sm)^(sum(dim)/2); %The normalization constant which is the area of 
                              % exp(-(x.*x + y.*y)/(2*sm*sm)) ....
 % CN=sum(sum(exp(-(x.*x + y.*y)/(2*sm*sm)))); 
%                  % %This gives a brut force way of getting the Normal Distribution to sum
                         % %up to unity. It gives slightly jaggier response
                         % for a fixed sinusoid when sm changes (scale
                         % space). 
 if (typ == 'gau')
    h = exp(-(x.*x + y.*y)/(2*sm*sm)); 
    h= h/CN;  %This produces the well known normal distribution.... 
              %...its area is 1 for continuous x,y 
end

if (typ == 'dxg')
    h = (x/sm/sm).*exp(-(x.*x + y.*y)/(2*sm*sm))/CN;
    %Implements approximation to analytic derivation,  with ramp, y=alfa*x, normalization. The expression is
    %analytic derivation of 2d Gaussian, assumed to be used in filtering (not convolution, which assumes reflection of filter). 
    %Thus, if the input is y=alfa*x  the output is ~alfa provided that x is an integer grid, and y is in R, unbounded.
    %Then, setting sm=0.905,  (assuming y is unbounded),  produces the best accuracy of alfa=1, (resulting in
    % 0.001 relative error on the integer grid =x). 
    %rmp=@(s) ((sum(abs((1/(s*s))*(1/sqrt(2*pi))*(1/s)*gaussgen(s,'xxg')))));
    %[ rmp(0.80) rmp(0.8009) rmp(0.805) rmp(0.807) rmp(0.8099) rmp(0.81) rmp(0.83) rmp(0.85) rmp(0.90) rmp(0.95)]
    %The relative error can be verified via:
    % sm=0.905;1-(gaussgen(sm,'dxg'))*[-round(pi*sm):round(pi*sm)]' 
    %
    % When  y is bounded (typical for sensors/cameras) on an integer x grid,  |alfa| is bounded too.  Using the same sm=0.905, and assuming y is discrete (256 levels), 
    % with |y|<=1, then alfa is bounded as,  |alfa|<= 127/3 resulting in the relative accuracy of  0.004 of the ideal alfa=127/3 in 1D:
    %xgr=[-round(pi*sm):round(pi*sm)];lv=256;alfa=(lv/2-1)/xgr(end);ymxalfa=round(alfa*xgr); g=gaussgen(sm,'dxg'); [g*ymxalfa' 1-g*ymxalfa'/alfa, g*ymxalfa'/(lv/2-1)]   .    
    %
    %AMPLIFICATION of max-norm: 
    %The (same 1x7) filter yields on the same 1D input (or 2D ramp)  the  amplification
    %of the max-norm of 0.332 (see above). The (same 1x7) filter yields by contrast 0.785 
    %as the amplification (of max-norm) if the input is the ideal 1d (or 2d) edge (plus/minus 127, 
    %assuming 256 gray levels):
    %sum(abs(127*gaussgen(sm,'dxg')))/(127) .
    %For a 1D or 2d sinusoid of omega=1/sm where sm=0.905, the input max-norm (the
    %amplitude) is amplified with factor 0.670 (=sm^(-1)*exp(-1/2) per analytic estimation) with the (same 1x7) filter.
    %For a 2d delta,  the input max-norm (the %amplitude) is amplified with
    %the factor 0.129 in 2D and the factor 0.292 in 2D, 
    %(x=round(sm);y=0; (x/sm/sm)* exp(-(x.*x)/(2*sm*sm))/sqrt(2*pi*sm*sm)*[exp(-( y.*y)/(2*sm*sm))/sqrt(2*pi*sm*sm),  1] per computation).
    
%   h = h/sum(sum(abs(h))); % %Brute force derivation constant that delivers 
%   half of the height of a  step function (taking values -1 and 1)
end  

if (typ == 'dxx')
    h = -((1/sm/sm)-(x/sm/sm).^2).*exp(-(x.*x + y.*y)/(2*sm*sm))/CN;
%   h = h/sum(sum(abs(h))); % %Brute force derivation constant that delivers 
%   half of the height of a  step function (taking values -1 and 1)
end


if (typ == 'xxg')
    h = x.*x.*exp(-(x.*x + y.*y)/(2*sm*sm));
%    h = h/sum(sum(abs(h)));
end

return



