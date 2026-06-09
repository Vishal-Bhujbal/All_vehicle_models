mux = 1.2543;
function [x,y,fxfl,fyfl,fxfr,fyfr,fxrl,fyrl,fxrr,fyrr] = gg_wheel()

muy = 1.27645;

normal = (Par.sprung_mass/4 + Par.unsprung_mass)*9.81;

theta = linspace(0,2*pi,200);
x = muy*cos(theta);
y = mux*sin(theta);


fxfl = out.Fx_fl.Data/(normal(1));
fyfl = out.Fy_fl.Data/(normal(1));

fxfr = out.Fx_fr.Data/(normal(2));
fyfr = out.Fy_fr.Data/(normal(2));

fxrl = out.Fx_rl.Data/(normal(3));
fyrl = out.Fy_rl.Data/(normal(3));

fxrr = out.Fx_rr.Data/(normal(4));
fyrr = out.Fy_rr.Data/(normal(4));
% hold("on")
% plot(x,y)

% scatter(fy,fx,'r.')
% hold("off")
end

gg