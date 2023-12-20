clc;clear;close all;
%% 
T05_1=[1 0 0 200
       0 1 0 0
       0 0 1 0
       0 0 0 1];
angle1=Inverse_kinematic(T05_1)

%% 
L(1)=Link('alpha',0,'a',0,'d',0,'modified');
L(2)=Link('alpha',-pi/2,'a',30,'d',25,'modified');
L(3)=Link('alpha',0,'a',240,'d',0,'modified');
L(4)=Link('alpha',0,'a',120,'d',0,'modified');
L(5)=Link('alpha',pi/2,'a',15,'d',20,'modified');
bot=SerialLink(L,'name','robot');
verify_angle1=bot.ikine(T05_1,'mask',[1 1 1 1 1 0])
verify_forward_angle1=bot.fkine(verify_angle1)

%%
q=[0 90 0 0 0];
T=forward_kinematic(q)

%%
function T=forward_kinematic(q)
for i=1:5
q(i)=q(i)*pi/180;
end
dh=[0 0 0 q(1);
    -pi/2 30 25 q(2);
    0 240 0 q(3);
    0 120 0 q(4);
    pi/2 15 20 q(5)
    ]
t_01=[cos(dh(1,4)) -sin(dh(1,4)) 0 dh(1,2);
    sin(dh(1,4))*cos(dh(1,1)) cos(dh(1,4))*cos(dh(1,1)) -sin(dh(1,1)) -sin(dh(1,1))*dh(1,3);
    sin(dh(1,4))*sin(dh(1,1)) cos(dh(1,4))*sin(dh(1,1)) cos(dh(1,1)) cos(dh(1,1))*dh(1,3);
    0 0 0 1
    ];
t_12=[cos(dh(2,4)) -sin(dh(2,4)) 0 dh(2,2);
    sin(dh(2,4))*cos(dh(2,1)) cos(dh(2,4))*cos(dh(2,1)) -sin(dh(2,1)) -sin(dh(2,1))*dh(2,3);
    sin(dh(2,4))*sin(dh(2,1)) cos(dh(2,4))*sin(dh(2,1)) cos(dh(2,1)) cos(dh(2,1))*dh(2,3);
    0 0 0 1
    ];
t_23=[cos(dh(3,4)) -sin(dh(3,4)) 0 dh(3,2);
    sin(dh(3,4))*cos(dh(3,1)) cos(dh(3,4))*cos(dh(3,1)) -sin(dh(3,1)) -sin(dh(3,1))*dh(3,3);
    sin(dh(3,4))*sin(dh(3,1)) cos(dh(3,4))*sin(dh(3,1)) cos(dh(3,1)) cos(dh(3,1))*dh(3,3);
    0 0 0 1
    ];
t_34=[cos(dh(4,4)) -sin(dh(4,4)) 0 dh(4,2);
    sin(dh(4,4))*cos(dh(4,1)) cos(dh(4,4))*cos(dh(4,1)) -sin(dh(4,1)) -sin(dh(4,1))*dh(4,3);
    sin(dh(4,4))*sin(dh(4,1)) cos(dh(4,4))*sin(dh(4,1)) cos(dh(4,1)) cos(dh(4,1))*dh(4,3);
    0 0 0 1
    ];
t_45=[cos(dh(5,4)) -sin(dh(5,4)) 0 dh(5,2);
    sin(dh(5,4))*cos(dh(5,1)) cos(dh(5,4))*cos(dh(5,1)) -sin(dh(5,1)) -sin(dh(5,1))*dh(5,3);
    sin(dh(5,4))*sin(dh(5,1)) cos(dh(5,4))*sin(dh(5,1)) cos(dh(5,1)) cos(dh(5,1))*dh(5,3);
    0 0 0 1
    ];
T=t_01*t_12*t_23*t_34*t_45;
end

%%
function angles=Inverse_kinematic(T_05)
sita5=atan2(T_05(3,2),-T_05(3,1));
sita1=atan2(T_05(2,3),T_05(1,3));
cos_phi=T_05(3,3);
sin_phi=T_05(1,3)/cos(sita1);
cos_sita3=( ...
    (T_05(1,4)/cos(sita1)-15*T_05(3,3)-20*T_05(1,3)/cos(sita1)-30+25*tan(sita1))^2+(20*T_05(3,3)-15*T_05(1,3)/cos(sita1)-T_05(3,4))^2-120^2-240^2 ...
    )/240^2
len1=240;len2=120;
if cos_sita3>=-1&&cos_sita3<=1
    sin_sita3=sqrt(1-cos_sita3^2);
    sita3_1=atan2(sin_sita3,cos_sita3);
    sita3_2=atan2(-sin_sita3,cos_sita3);
    if 20*T_05(3,3)-15*T_05(1,3)/cos(sita1)-T_05(3,4)==0&T_05(1,4)/cos(sita1)-15*T_05(3,3)-20*T_05(1,3)/cos(sita1)-30+25*tan(sita1)==0
        sita2_1=0;
        sita2_2=0;
        sita4_1=atan(sin_phi,cos_phi)-sita2_1-sita3_1;
        sita4_2=atan(sin_phi,cos_phi)-sita2_2-sita3_2;
    else
        k1_1=len1+len2*cos_sita3;
        k2_1=len2*sin_sita3;
        sita2_1=atan2(20*T_05(3,3)-15*T_05(1,3)/cos(sita1)-T_05(3,4),T_05(1,4)/cos(sita1)-15*T_05(3,3)-20*T_05(1,3)/cos(sita1)-30+25*tan(sita1))-atan2(k2_1,k1_1);
        sita4_1=atan2(sin_phi,cos_phi)-sita2_1-sita3_1;

        k1_2=len1+len2*cos_sita3;
        k2_2=len2*(-sin_sita3);
        sita2_2=atan2(20*T_05(3,3)-15*T_05(1,3)/cos(sita1)-T_05(3,4),T_05(1,4)/cos(sita1)-15*T_05(3,3)-20*T_05(1,3)/cos(sita1)-30+25*tan(sita1))-atan2(k2_2,k1_2);
        sita4_2=atan2(sin_phi,cos_phi)-sita2_2-sita3_2;
    end
    angles=[sita1,sita2_1,sita3_1,sita4_1,sita5;sita1,sita2_2,sita3_2,sita4_2,sita5];
else
    angles=NaN(2,5);
end
end
