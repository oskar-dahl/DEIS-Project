function [hout, x, y]= symdergaussgen3(typ,sm,n,sizev,verbose,normalize)
% $Id: symdergaussgen.m,v 1.4 2014/01/25 22:43:35 josef Exp $
% (C) Josef Bigun
% This function generates a symmetry derivative filter with
%          standard deviation of  sm,  symmetry order type (an integer).
%These filters are described in
%           Bigun J. "Vision with direction", Springer, (2006)
%in further detail. They are of the form
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           r^n exp( -(r/sm)^2/2 ) exp( i typ fi )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%except for normalization where n, is positive and controls (together with sm)
%           the effective bandwith, and the radial location of the peak magnitude of the thorus.
%
%OBS: when typ=0, the function outputs the thorus to ease the possibility
%of generating linear symmetry filters with outer scale filter being a
%thorus.  If the outer-scale of linear symmetry filters must be a pure Gaussian
%in an application (which the definition of symderivative filter in the reference
%boils down to a gaussian),  then use gaussgen.m instead.
%
% When sm is negative,  its value is interpreted as r0, and the according sm is calculated. 
%
% If an element of  sizev is negative, then the filter size in the
% negative dimensions are calculated by the routine itself.
%
% verbose=0 generates no information whereas verbose=1 generates
%information on the various details of the filter, currently two pictures of it
%(magnitude, and the complex coefficients in color), and a  1D graph of its
%magnitude along the x-axis. The %figure handles are then counted up from the current
%%figure handle, at the time of calling; If verbose is a vector of dimension 3, then the %figure
%handles are set to the contents of verbose;
%
% normalize=1 (default) generates a filter that is normalized with 1-norm. If
% normalize=n the filters are normalized such that its n-norm  is
% 1, etc. If normalize=0, the filter is normalized with infinite (max)
% norm.
%
% The function can be called with 0, 1, 2, 3, 4 and 5 arguments as:
%     symdergaussgen, symdergaussgen(type), symdergaussgen(typ,sm),
%     symdergaussgen(typ,sm,n), and symdergaussgen(typ,sm,n,sizev),
%     symdergaussgen(typ,sm,n,sizev,normalize).
% The default values of the omitted variables  are
% typ=1; sm=1; n=1; sizev=[-1,-1]; verbose=1; normalize=1;



%%Argument parsing, and defaults
if nargin<1  typ=1;end          %0 arguments
if nargin <2 sm=1;  end         %0,1 arguments
if nargin<3  n=1.0; end     %0,1,2 arguments
if nargin<4  sizev=[-1,-1]; end %0,1,2,3 arguments
if nargin<5  verbose=1; end     %0,1,2,3,4 arguments
if nargin<6  normalize=1; end     %0,1,2,3,4,5 arguments

atyp=abs(typ);

if verbose
    %disp(['Generating symmetry derivative filter with parameters...']);
    %disp(['typ=' num2str(typ) '; sm=' num2str(sm) '; n=' num2str(n) '; sizev=[' num2str(sizev) ']; verbose=' num2str(verbose)]);
end

if sm<0
    r0=abs(sm);
    sm= r0/sqrt(n); %if sm is negative then it is interpreted as r0
end;

EF=sm*(prod(1./[1:2:(atyp-1)]+1))*sqrt(2/pi);
if mod(atyp,2)==1
    EF=sm*(prod(1./[2:2:(atyp-1)]+1))*sqrt(pi/2);
end
%if verbose(1)>0
    %Infos=['Radius at filter maximum r0=', num2str(r0)];
    %disp(Infos)
    %Infos=['EF=', num2str(EF)]; %Expected...
   %disp(Infos)
%end

%The filter magnitude
hh=@(rr) ((rr.^n).*exp(-(rr/sm).^2/2)/(((sqrt(n)*sm)^n)*exp(-n/2)));

%Calculate filter size as the root of ep-H
%which is the function  formed to represent the filter magnitude  at truncation,
ep=exp(-9/sqrt(2));
H=@(rr) ep-hh(rr);
rn=fzero(H,sqrt(n)*sm*[0, 1]);
rx=fzero(H,sqrt(n)*sm*[1, 20]);
wr=rx-rn;
if verbose(1)>0
    %Infos=['Inner radius, rn, and Outer radius, rx, are defined as the filter values reaching ep=', num2str(ep) ];disp(Infos)
    %Infos=['rn=',num2str(rn), '  rx=',num2str(rx), '  rx-rn=',num2str(wr) ];disp(Infos)
end

if sizev(1)<0 sizev(1)=1+2*round(rx); end
if sizev(2)<0 sizev(2)=1+2*round(rx); end

[x,y]=meshgrid(-(sizev(2)-1)/2:(sizev(2)-1)/2,-(sizev(1)-1)/2:(sizev(1)-1)/2);
[th,r]=cart2pol(x,y);


h=hh(r);
%At this point max of the (abs) filter is 1

if 0<normalize
    h = h/(sum(sum(h.^normalize)))^(1/normalize);
end

[hre,him]=pol2cart(typ*th,h);
hout=hre+i*him;

if verbose(1)>0
    Infos=['filtersize=', num2str(size(h))];disp(Infos)
%     if isempty(get(groot,'Current%figure'))
%     %figure(verbose(1));
%     end
% get(gcf,'Number'); %Note: This creates an extra %figure(1) if there is no current %figure
    fhi20=verbose(1);
    fhi11=fhi20+1;
    fhi11_1d=fhi20+2;
    if length(verbose)>1
        fhi11=verbose(2);
        fhi11_1d=verbose(3);
    end
    %figure(fhi11);
    %lsdisp(complex(h),1,1)
    %figure(fhi20)
    %lsdisp(hout,1,1)
end

if verbose(1)>0
    %figure(fhi11_1d);
    if size(h,1) <= size(h,2)
        %plot(x((size(h,1)+1)/2,:),h((size(h,1)+1)/2,:))
    else
        %plot(y(:,(size(h,2)+1)/2),h(:,(size(h,2)+1)/2) )
    end
    %axis tight
end

return





