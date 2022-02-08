clear all; close all; clc;


load('nominalBeziersHuman.mat')

global Ts g bv_zcom useConstant meanZcom
Ts = 0.8;
g = 9.81;

bv_zcom = nominalBeziers.bv_zcom;
bv_xcom = nominalBeziers.bv_xcom;
bv_dxcom = nominalBeziers.bv_dxcom;
x0 = [bezier2(bv_xcom,0.0);
      bezier2(bv_dxcom,0.0)];
meanZcom = mean(bezier2(bv_zcom,0:0.01:1));


useConstant = false;
[t,x] = ode45(@eom,[0 Ts],x0)
figure; 
subplot(1,2,1); hold on; grid on;
plot(t,x(:,1))
subplot(1,2,2); hold on; grid on;
plot(t,x(:,2))

useConstant = true;
[t,x] = ode45(@eom,[0 Ts],x0)
subplot(1,2,1); hold on; grid on;
plot(t,x(:,1))
plot(0:0.01:Ts,bezier2(bv_xcom,0:0.01:Ts))
subplot(1,2,2); hold on; grid on;
plot(t,x(:,2))
plot(0:0.01:Ts,bezier2(bv_dxcom,0:0.01:Ts))
legend('non-constant','constant','beziers')



%% Functions
function dx = eom(t,x)
    global Ts g bv_zcom useConstant meanZcom
    if useConstant
        dx = [x(2);
              sqrt(g/meanZcom)*x(1)];
    else
        dx = [x(2);
              sqrt(g/z(t))*x(1)];
    end
end

function eval = z(t)
    global Ts bv_zcom
    t_norm = t/Ts;
    eval = bezier2(bv_zcom,t_norm);
end




