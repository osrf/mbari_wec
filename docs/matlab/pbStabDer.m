%
% PURPOSE: Estimate stability derivatives for the buoy, PTO and heave cone
%          components of the Power Buoy.
%
% DATE:    20 Dec 2022
%
% NOTES:  The stability derivatives are defined with respect to land robot
% coordinates, with z up and x towards the front.  The origin is at the pivot.
%
% REFERENCES:
% (1) Francois C, Solid Works schematic of "MBARI Wave Energy Converter: Heave 
%     Cone", April, 2022.
% (2) Francois C, Solid Works schematic of "MBARI Wave Energy Converter: Power
%     Take Off Device", Dec. 5, Jan 6, 2021.
% (3) Francois C, Solid Works schematic of "MBARI Wave Energy Converter: Buoy",
%     10/11/2022.
% (4) Hoerner, "Fluid-Dynamic Drag", 1965.
% (5) Blevins, "Formulas for Dynamics, Acoustics and Vibration", Wiley, 2016.
%
% Parameters:
%
  rho = 1025;                %kg/m^3 Density of sea water
%
% ***   Buoy   ***
%
  dia_b = 2.64;                %m
  h_b   = 1.12-.81;            %m height of the beveled section [3]
  m_b     = 1400;              %kg Mass of the buoy (in air, no water).
  m_w_pto = 395;               %kg This is in-water weight / g
  m_w_hc  = 703;               %kg This is in-water weight / g
  Cdp     = 1.17;              %none.  For a flat plate, 3 dim perp flow.
  Ixx_b   = 1492;
  Ixy_b   = -14.15;
  Ixz_b   = 12.96;
  Iyy_b   = 1539;
  Iyz_b   = -14.59;
  Izz_b   = 650.5;
%  
% Approximate the below-water part of the buoy as a hemisphere.  Find the radius
% through the density of the displaced water.
%
% Vol of a hemisphere = Vh = 2/3*pi*r^3.              (1)
%
% Vol of displaced sea water = 
%
  Vw = (m_b + m_w_pto + m_w_hc)/rho;
%
% where rho*(m_w_pto and m_w_hc) is the in-water weight of the pto and 
% heave cone respectively.  I'm using in-water weights for these two because
% we want the displaced volume of the buoy, not the displaced volume of the
% whole rig.   Then, from Archimedes:
%
% Vh = Vw
%
% Substituting (1) for Vh above and solving for r gives
%
  r = ( 3/2/pi*Vw )^(1/3);
%
% Now the added mass of a 1/2 submerged sphere is (Blevins, #10, page 326)
%
  Xudot_b = (2/3) * (1/2) * rho * pi * r^3;
%
  Yvdot_b = Xudot_b;
  Zwdot_b = 8/3*rho*(dia_b/2)^3;            %kg.  Blevins p.325, #1.
%
% Find the drag by assuming a flat plate with area equal to the side view of 
% the in-water part.  This is a trapezoid with base b1  = dia_b.  Assume the
% sides form 45 degree angles, then:
%
  b1 = dia_b;
  b2 = dia_b - 2*h_b;
  At = 1/2 * h_b * (b1 + b2);

  Xuabsu_b = -1/2*rho*Cdp*At;   %kg/m. 
  Yvabsv_b = Xuabsu_b;
%
% Flat plate again, with area of a circle:
%
  a        = dia_b/2;
  Zww_b    = -1/2*rho*Cdp*pi*a^2;           %kg/m, down only.
%
% Now build the generalized mass matrix, about the COW:
%
  M_b = [ m_b     0      0     0     0     0;
          0      m_b     0     0     0     0;
          0       0     m_b    0     0     0;
          0       0      0    Ixx_b Ixy_b Ixz_b;
          0       0      0    Ixy_b Iyy_b Iyz_b;
          0       0      0    Ixz_b Iyz_b Izz_b ];
%
% Transform it to the bridle point using Joan's generalized parallel axis theorem:
%
  MT_b = parallelAxis( M_b, [ 0  0  -2.27]' );
% MT_b = parallelAxis( M_b, [ 0  0  -3.20]' );
        
%
% ***   PTO  ***
%
  l_pto = 8.82;                          %m, total length not including tether, L.
  d_pto = .21;                           %m, diameter, D, scaled from .34 m, [2].
  m_pto = 605;                           %kg, mass (in air).
  Cd_pto= 1.2;                           %none. Section drag of a cylinder for
                                         % 1e4 < Re < 1.3e5
%
% Translation:
%
% Added mass of a cylindrical section; Blevins #1 p. 322.
%
  Xudot_pto = rho*pi*d_pto^2/4*l_pto;    %kg
  Yvdot_pto = rho*pi*d_pto^2/4*l_pto;    %kg
  Zwdot_pto = 0;
%  
% Drag of cylindrical section Re ~ 10^4
%
  Xuabsu_pto = -1/2*rho*l_pto*d_pto*Cd_pto; %kg/m, Cd ~= 1.0 at Re ~= 10,000.
  Yvabsv_pto = Xuabsu_pto;
  Zwabsw = 0;
%
% Rotation:
%
% Model the pto as a long, thin, vertical cylinder, pivoting at the top.  It is
% length L, and a distance down the cylinder is l meters from the top.
%
% drag of a thin cross section at a point l is then
%
% Drag = 1/2 * rho * (l*q)^2 * Cd * D * dl
%
% where q is the angular speed.  dl is an infinitesimal distance along
% the cylinder.  The moment contribution from dl is then D*l.  Adding all these
% up gives a total moment of
%
%             L
% M = integral   1/2 * rho * (l*q )^2 * Cd * D * l * dl.
%             0
% Performing the integration gives:
%
% M = rho/8 * q^2 * D * L^4 * Cd
%
% Accounting for the sign:
% M = -rho/8 * D * L^4 * Cd * q * |q|
%
% So then:
  Kpabsp_pto = -rho/8 *d_pto*l_pto^4*Cd_pto; %kg m^2.  Assuming origin at the top.
  Mqabsq_pto = Kpabsp_pto;               %kg m^2.  
  Zrabsr_pto = 0;
%  
% ***  Heave Cone  ***
%
  A_c       = 6.56;          %m^2 Projected area, doors closed, along z.
  A_o       = 5.14;          %m^2 Projected area, doors open, along z.
  w_hc      = 2.8;           %m,  Width of the heave cone, [1]
  hs_hc     = 1.3;           %m,  height of the stem.  See drawing.
  h_hc      = .4;            %m,  height of the heave cone.
  A_x       = w_hc-h_hc^2;   %m^2 Frontal area (trapezoid), perpendicular to x.
  Cd_hc     = 1.2;           %none. Drag of cylindrical section.  
%  
% Translation: 
%
% Doors closed, moving up (*_c_u).  Drag specified in [1].  Checks with the 3D
% drag of a cup, from Hoerner p. 3-17 Fig. 32 b.
%
  Zwdot_c_u = 10000;                     %kg,  Francois [1].
  Zww_c_u   = -1/2*rho*1.45*A_c;         %kg/m  
%  
% Doors closed, moving down (*_c_d).  
  Zwdot_c_d = Zwdot_c_u;                 %kg For now. Probably less
  Zww_c_d   = 1/2*rho*.8*A_c;            %kg/m Francois [1]
% Doors open, moving up (*_o_u).  
  Zwdot_o_u = 3000;                      %kg  Francois [1].
  Zww_o_u   = -1/2*rho*1.2*A_c;          %kg/m  Francois [1].
% Doors open, moving down (*_o_d).  
  Zwdot_o_d =  Zwdot_o_u;                %kg For now. Probably less
  Zww_o_d   = 1/2*rho*.85*A_c;          %kg/m Francois [1].
%
% For accel in the x direction (parallel to bottom plate), we will approximate
% the heave cone as a rectangular solid with the same dimensions as the heave
% cone. See Blevins p.325 #6 for a  rectangular solid:
  a = w_hc;
  c = a;
  b = h_hc;
  alpha = .64*sqrt(b*c)/a;
  Xudot_hc  = alpha*rho*a*b*c;           %kg
  Xuabsu_hc = -1/2*rho*A_x*Cdp;         %kg/m Flat plate perp to flow
%
  Yvdot_hc  = Xudot_hc;                  %kg
  Yvabsv_hc = Xuabsu_hc;                 %kg/m
%
% Rotation:
%
% Compute the torque necessary to rotate the heave cone about its pivot
% perpendicular to the stem.  This is like swinging an umbrella by its
% handle. I will superimpose the drag of two pieces; the edge-on cone, as we
% have done before, and then a hexagonal plate rotating about its center line.
%
% The drag of the edge-on cone (tangential direction to the rotation) is
% 
% D = -1/2 * rho * (hs_hc * q)^2 * Cdp * A_xc.
%
% M = D * hs_hc.
%
% where hs_hc is also the moment arm.  So:
%
  Mqabsq_tang  = -1/2*rho*hs_hc^3*Cdp*A_x;
%
% Now find the moment of an octagonal plate twirling about its centerline. 
% First, the half-plate above the centerline is a trapezoid with a long base
% twice the length of the short base.  See [1].  The height is such that this
% trapezoid is composed of three equilateral triangles.  The total width D of
% the plate as a function of height h above the centerline is then
%
% D(h) = 2/sqrt(3) * ( 2*R - h ).      (2)
%
% The moment of an infinitesimally thin strip parallel to the centerline is
% then: 
%
% dM = 1/2 * rho * (h*q)^2 * Cd * D(h) * dh * h
%
% where the final h is the moment arm.  We'll take Cd to be similar to a flat
% plate section.  Then substituting (2) into the above,
%
%                                                R
% M = 1/2 * rho * Cd * q^2 * 2/sqrt(3) * integral (2*R*h^3 - h^4) * dh
%                                                0
% Peforming the integration, with R = w_hc/2,  gives:
% 
  Mqabsq_rot = -1/2*rho*sqrt(3)/5*(w_hc/2)^5*Cd_hc; %kg m^2
%
  fprintf('Mq|q| rotat   [kg m^2]: %12.2f  \n', Mqabsq_rot );
  fprintf('Mq|q| tang    [kg m^2]: %12.2f  \n', Mqabsq_tang);
  Mqabsq_hc = Mqabsq_rot + Mqabsq_tang; %kg m^2
  Kpabsp_hc = Mqabsq_hc;                %kg m^2

% Approx area of panel:
  A_p = (2.8/sqrt(3) -.2/sqrt(2))*.4;    %m^2
  A_pt = 6*A_p;                          %m^2 total area
%
% Approximate this one as a flywheel?
%
% Nrabsr_hc =                           %kg m^2


fprintf('\n** Buoy **\n\n');  

fprintf('Xudot           [kg]: %12.2f  \n',Xudot_b);
fprintf('Xu|u|         [kg/m]: %12.2f  \n',Xuabsu_b);

fprintf('Yvdot           [kg]: %12.2f  \n',Yvdot_b);
fprintf('Yv|v|         [kg/m]: %12.2f  \n',Yvabsv_b);

fprintf('These values are for downward motion.\n');
fprintf('Zwdot           [kg]: %12.2f  \n',Zwdot_b);
fprintf('Zww           [kg/m]: %12.2f  \n',Zww_b);

fprintf('\n** Power Take Off **\n\n');  

fprintf('Xudot           [kg]: %12.2f  \n',Xudot_pto);
fprintf('Xu|u|         [kg/m]: %12.2f  \n',Xuabsu_pto);
fprintf('Kp|p|       [kg m^2]: %12.2f  \n',Kpabsp_pto);

fprintf('Yvdot           [kg]: %12.2f  \n',Yvdot_pto);
fprintf('Yv|v|         [kg/m]: %12.2f  \n',Yvabsv_pto);
fprintf('Mq|q|       [kg m^2]: %12.2f  \n',Mqabsq_pto);

fprintf('\n** Heave Cone **\n\n');  
fprintf('Doors Open, Moving Up:\n');
fprintf('Zwdot           [kg]: %12.2f  \n',Zwdot_o_u);
fprintf('Zww           [kg/m]: %12.2f  \n',Zww_o_u);
fprintf('Doors Open, Moving Down:\n');
fprintf('Zwdot           [kg]: %12.2f  \n',Zwdot_o_d);
fprintf('Zww           [kg/m]: %12.2f  \n',Zww_o_d);
fprintf('Doors Closed, Moving Up:\n');
fprintf('Zwdot           [kg]: %12.2f  \n',Zwdot_c_u);
fprintf('Zww           [kg/m]: %12.2f  \n',Zww_c_u);
fprintf('Doors Closed, Moving Down:\n');
fprintf('Zwdot           [kg]: %12.2f  \n',Zwdot_c_d);
fprintf('Zww           [kg/m]: %12.2f  \n',Zww_c_d);
fprintf('\n');
fprintf('Xudot           [kg]: %12.2f  \n',Xudot_hc);
fprintf('Xu|u|         [kg/m]: %12.2f  \n',Xuabsu_hc);
fprintf('Yvdot           [kg]: %12.2f  \n',Yvdot_hc);
fprintf('Yv|v|         [kg/m]: %12.2f  \n',Yvabsv_hc);
fprintf('Kp|p|       [kg m^2]: %12.2f  \n',Kpabsp_hc);
fprintf('Mq|q|       [kg m^2]: %12.2f  \n',Mqabsq_hc);
fprintf('Nr|r|       [kg m^2]: %12.2f  \n',NaN);

%
% Skew-symmetric operator defined below.  See Fossen 2.6 on p 8.
%
function M = S(x) 
  if(size(x) ~= [3 1])
    fprintf('S(x) - Error.  x must be 3x1.\n');
    M = 0*eye(3);
    return;
  end
  M = [  0      -x(3)     x(2);
        x(3)      0      -x(1);
       -x(2)     x(1)      0   ]; 
end
%
% Joan's parallel axis theorem:
%
function  MT = parallelAxis(M, r)
  if(size(M) ~= [6 6])
    fprintf('Joan - Error.  M must be 6x6.\n');
    MT = 0*eye(6);
    return
  end
  if(size(r) ~= [3 1])
    fprintf('Joan - Error.  r must be 3x1.\n');
    MT = 0*eye(6);
    return
  end
  M11 = M(1:3,1:3);
  M12 = M(1:3,4:6);
  M21 = M(4:6,1:3);
  MT = [ 0*eye(3)             -M11*S(r);
         S(r)*M11     -S(r)*M11*S(r)+S(r)*M12-M21*S(r) ];
  MT = MT + M;
end
