function cannonFunction(app)
%{
Does the math
%}
%% Set Parameters
cdrag = 0.47;
g = 9.80665; % m/s^2
m = 0.002; % kg
rho = 1.204; % kg/m^3
A = pi*.0001125; % m^2

vinit = 2.02; % m/s

c = (cdrag*rho*A)/(2*m); % Creates a constant for easier calculations

%% Obtain Target Position
xfinal = app.HorizontalDistanceSlider.Value;
yfinal = app.VerticalDistanceSlider.Value;
%% Set up for loops
Nt = 10000; % number of time points
Tf = 5; % seconds

t = linspace(0, Tf, Nt);
deltat = t(2) - t(1);

decimalplaces = 2; % how many decimal places to round to: 3 = mm, 2 = cm, etc.

theta = 89.9; % degrees- intializing variable
deltaTheta = 0.1; % How much we change theta by after each loop

x = zeros(1, Nt);
xvel = zeros(1, Nt);

y = zeros(1, Nt);
y(1) = .12; % m
yvel = zeros(1, Nt);

timemark = 0;
solutionFound = false;

%% Run the loop
while theta > 0
    
    % Creates initial velocities
    xvel(1) = vinit * cosd(theta);
    yvel(1) = vinit * sind(theta);
    
    % Creates values for the motion of the projectile- Verlet Method
    for it = 1:Nt-1
        vel = sqrt(xvel(it)^2 + yvel(it)^2);
        phi = atan(yvel(it)/xvel(it));
        dragAccel = c*vel^2;
        xdragAccel = cos(phi)*dragAccel;
        ydragAccel = sin(phi)*dragAccel;
        
        xvelhalf = xvel(it) - xdragAccel * deltat/2;
        x(it+1) = x(it) + xvelhalf * deltat;
        
        yvelhalf = yvel(it) + ((-g - (yvel(it)/abs(yvel(it)))*ydragAccel) * deltat/2);
        y(it+1) = y(it) + yvelhalf * deltat;
        
        velhalf = sqrt(xvelhalf^2 + yvelhalf^2);
        halfphi = atan(yvelhalf/xvelhalf);
        halfdragAccel = c*velhalf^2;
        xhalfdragAccel = cos(halfphi)*halfdragAccel;
        yhalfdragAccel = sin(halfphi)*halfdragAccel;
        
        xvel(it+1) = xvelhalf - xhalfdragAccel * deltat/2;
        
        yvel(it+1) = yvelhalf + ((-g - (yvelhalf/abs(yvelhalf))*yhalfdragAccel) * deltat/2);
    end
    
    % Checks to see when x = xfinal, then checks if y = yfinal at same time
    for i = 1:Nt
        if ((round(xfinal, decimalplaces) == round(x(i), decimalplaces)) && (round(yfinal, decimalplaces) == round(y(i), decimalplaces)))
            solutionFound = true;
            timemark = i;
        end
    end
    
    % Checks to see if the correct angle is found- if so, ends loop
    if solutionFound
        break
    else
        theta = theta - deltaTheta;
    end
end

%% Test results
if solutionFound
    plot(app.ProjectileAxes, x, y, 'b-');
    grid (app.ProjectileAxes, 'on');
    axis(app.ProjectileAxes, [0, xfinal+.1, 0, max(y(1:timemark))+.1])
    xlabel(app.ProjectileAxes, 'Distance (m)');
    ylabel(app.ProjectileAxes, 'Height (m)');
    title(app.ProjectileAxes, 'Flight Path');
    app.AngleNumber.Text = num2str(theta);
    app.TimeNumber.Text = num2str(timemark*Tf/Nt);
else
    app.AngleNumber.Text = 'N/A';
    app.TimeNumber.Text = 'N/A';
end

