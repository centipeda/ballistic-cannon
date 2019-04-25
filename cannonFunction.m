function [thetaVerlet, thetaClosed] = cannonFunction(app)
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
vTerm = sqrt((2*m*g)/(cdrag*A*rho)); % Calculates terminal velocity of projectile

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

yInitial = .12; % m
xInitial = 0;

x = zeros(1, Nt);
xvel = zeros(1, Nt);

y = zeros(1, Nt);
yvel = zeros(1, Nt);
y(1) = yInitial;

xClosed = zeros(1, Nt);
yClosed = zeros(1, Nt);
velClosed = 0;
yClosed(1) = yInitial;


timemarkVerlet = 0;
timemarkClosed = 0;
solutionFoundVerlet = false;
solutionFoundClosed = false;

thetaClosed = 0;
thetaVerlet = 0;

%% Run the loop
while theta > 0
    
    % Creates initial velocities
    xvinit = vinit * cosd(theta);
    yvinit = vinit * sind(theta);
    xvel(1) = xvinit;
    yvel(1) = yvinit;
    
    % Creates values for the motion of the projectile- Verlet Method
    for it = 1:Nt-1
        % Verlet Method
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
        
        % Closed Form
        time = it*Tf/Nt;
        vAsc = vTerm*((yvinit - vTerm*tan(time*g/vTerm))/(vTerm + yvinit*tan(time*g/vTerm)));
        if vAsc > 0
            yClosed(it+1) = yInitial + ((vTerm)^2)/(2*g)*log((yvinit^2 + vTerm^2)/(vAsc^2 + vTerm^2));
            ascTime = time;
            ascIndex = it;
        else
            yClosed(it+1) = yClosed(ascIndex) - vTerm*(time - ascTime);
        end
        xClosed(it+1) = (vTerm^2)/g* log((vTerm^2 + g*xvinit*time)/vTerm^2);
    end
    
    % Checks to see when x = xfinal, then checks if y = yfinal at same time
    % Verlet Method
    if ~solutionFoundVerlet
        for i = 1:Nt
            if ((round(xfinal, decimalplaces) == round(x(i), decimalplaces)) && (round(yfinal, decimalplaces) == round(y(i), decimalplaces)))
                solutionFoundVerlet = true;
                timemarkVerlet = i;
                thetaVerlet = theta;
            end
        end
    end
    
    % Closed Form
    if ~solutionFoundClosed
        for i = 1:Nt
            if ((round(xfinal, decimalplaces) == round(xClosed(i), decimalplaces)) && (round(yfinal, decimalplaces) == round(yClosed(i), decimalplaces)))
                solutionFoundClosed = true;
                timemarkClosed = i;
                thetaClosed = theta;
            end
        end
    end
    
    % Checks to see if the correct angle is found- if so, ends loop
    if solutionFoundVerlet && solutionFoundClosed
        break
    else
        theta = theta - deltaTheta;
    end
end

%% Test results
if solutionFoundVerlet && solutionFoundClosed
    plot(app.ProjectileAxes, x, y, 'b-');
    grid (app.ProjectileAxes, 'on');
    axis(app.ProjectileAxes, [0, xfinal+.1, 0, max(y(1:timemark))+.1])
    xlabel(app.ProjectileAxes, 'Distance (m)');
    ylabel(app.ProjectileAxes, 'Height (m)');
    title(app.ProjectileAxes, 'Flight Path');
    app.AngleNumberVerlet.Text = num2str(thetaVerlet);
    app.TimeNumberVerlet.Text = num2str(timemarkVerlet*Tf/Nt);
    app.AngleNumberClosed.Text = num2str(thetaClosed);
    app.TimeNumberClosed.Text = num2str(timemarkClosed*Tf/Nt);
else
    theta = 0;
    app.AngleNumberVerlet.Text = 'N/A';
    app.TimeNumberVerlet.Text = 'N/A';
    app.AngleNumberClosed.Text = 'N/A';
    app.TimeNumberClosed.Text = 'N/A';
end

