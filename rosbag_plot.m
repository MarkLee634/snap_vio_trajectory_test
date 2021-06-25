%% load data

experiment = 4;

if experiment == 1
    vicon_bag = rosbag("circle_1_T3s.bag");
    vicon_start_index = 12;
    est_start_index = 7;
elseif experiment == 2
    vicon_bag = rosbag("circle_1_T3s_vioqc.bag");
    vicon_start_index = 13;
    est_start_index = 5;
elseif experiment == 3
    vicon_bag = rosbag("circle_2_T5s.bag");
    vicon_start_index = 1688;
    est_start_index = 8367;
elseif experiment == 4
    vicon_bag = rosbag("circle_2_T5s_vioqc.bag");
    vicon_start_index = 1378;
    est_start_index = 6874;
else
    return    
end
    

vicon_odom_bag = select(vicon_bag,'Topic',"/dragonfly4/odom");
est_odom_bag = select(vicon_bag,'Topic',"/dragonfly4/quadrotor_ukf/control_odom");
est_pos = odom_2_pos(est_odom_bag);
[est_rot, est_rot_time] = odom_2_rotm_and_time(est_odom_bag);
vicon_pos = odom_2_pos(vicon_odom_bag);
[vicon_rot, vicon_rot_time] = odom_2_rotm_and_time(vicon_odom_bag);

%% sync time

% sync vicon from index 12, est from index 7  [1_T3s.bag]
% sync vicon from index 13, est from index 5  [1_T3s_vioqc.bag]
% sync vicon from index 1688, est from index 8367  [2_T5s.bag]
% sync vicon from index 2300, est from index 11504  [2_T5s_vioqc.bag]

% size(vicon_pos.x)
vicon_pos_syn = trimData(vicon_pos, vicon_start_index);
est_pos_syn = trimData(est_pos, est_start_index);
vicon_rot_syn = trimDataRot(vicon_rot, vicon_rot_time, vicon_start_index);
est_rot_syn = trimDataRot(est_rot, est_rot_time, est_start_index);


% size(vicon_pos_syn.time)
est_pos_syn_down_x = [];
est_pos_syn_down_y = [];
est_pos_syn_down_z = [];
est_pos_syn_down_time = [];
est_rot_syn_down_rotMat = [];
est_rot_syn_down_time = [];

for i = 1:1:size(vicon_pos_syn.time,1)
    
    [minVal, index ] = min(abs(vicon_pos_syn.time(i) - est_pos_syn.time)); %pos and rot same index
    temp_est_pos_syn_down.x = est_pos_syn.x(index); %get pos
    temp_est_pos_syn_down.y = est_pos_syn.y(index);
    temp_est_pos_syn_down.z = est_pos_syn.z(index);
    temp_est_pos_syn_down.time = est_pos_syn.time(index);
    
    temp_est_rot_syn_down.rotMat = est_rot_syn.rotMat(:,:,index); %get rot
    temp_est_rot_syn_down.time = est_rot_syn.time(index);
    
    
    est_pos_syn_down_x = [est_pos_syn_down_x; temp_est_pos_syn_down.x]; %concat
    est_pos_syn_down_y = [est_pos_syn_down_y; temp_est_pos_syn_down.y];
    est_pos_syn_down_z = [est_pos_syn_down_z; temp_est_pos_syn_down.z];
    est_pos_syn_down_time = [est_pos_syn_down_time; temp_est_pos_syn_down.time];
    
    est_rot_syn_down_rotMat(:,:,i) = temp_est_rot_syn_down.rotMat; %concat
    est_rot_syn_down_time = [est_rot_syn_down_time; temp_est_rot_syn_down.time]; 
    
end

est_pos_syn_down.x = est_pos_syn_down_x;
est_pos_syn_down.y = est_pos_syn_down_y;
est_pos_syn_down.z = est_pos_syn_down_z;
est_pos_syn_down.time = est_pos_syn_down_time;
est_rot_syn_down.rotMat = est_rot_syn_down_rotMat;
est_rot_syn_down.time = est_rot_syn_down_time;

% size(vicon_pos_syn.time)
% size(est_pos_syn_down.time)

%% set vicon init frame to 0,0
vicon_pos_syn_init.x = vicon_pos_syn.x(1);
vicon_pos_syn_init.y = vicon_pos_syn.y(1);
vicon_pos_syn_init.z = vicon_pos_syn.z(1);

vicon_pos_syn.x = vicon_pos_syn.x - vicon_pos_syn_init.x;
vicon_pos_syn.y = vicon_pos_syn.y - vicon_pos_syn_init.y;
vicon_pos_syn.z = vicon_pos_syn.z - vicon_pos_syn_init.z;
%% offset data to same initial frame (Translation & Rotation!!!)
% est - vicon = offset
% est_off = est - offset
offset_pos = getOffsetPos(est_pos_syn_down, vicon_pos_syn);
est_pos_syn_down_offset = subtractOffset(est_pos_syn_down, offset_pos);

%%

% get Rotation offset
offset_rotation = getOffsetRot( est_rot_syn_down, vicon_rot_syn);
quat = rotm2quat(offset_rotation.rotMat)
% est_rot_syn_down_offset = MultRotOffset(offset_rotation, est_rot_syn_down); 

%%
%test quaternion
% quat = quaternion(1, 0.003, 0.003, 0.008); %x 30, y0, z0
% quat = quaternion(1, 0, 0, 0); %x 30, y0, z0
% quat = quaternion(0.966, 0, 0.259, 0); %x 30, y0, z0
rotq = quat2rotm(quat);

transformedPos.x = [];
transformedPos.y = [];
transformedPos.z = [];
% xyz transformation
for i=1:1:size(est_pos_syn_down_offset.x)
%   temp  = est_rot_syn_down_offset.rotMat(:,:,i) * [est_pos_syn_down_offset.x(i); est_pos_syn_down_offset.y(i); est_pos_syn_down_offset.z(i)];
  temp  =  rotq * [est_pos_syn_down_offset.x(i); est_pos_syn_down_offset.y(i); est_pos_syn_down_offset.z(i)];
  
  transformedPos.x = [transformedPos.x; temp(1)];
  transformedPos.y = [transformedPos.y; temp(2)];    
  transformedPos.z = [transformedPos.z; temp(3)];
end


% check 0,0,0 for initial difference bc offset
% errorOffset = [transformedPos.x(1) - vicon_pos_syn.x(1); transformedPos.y(1) - vicon_pos_syn.y(1); transformedPos.z(1) - vicon_pos_syn.z(1)]



%% RMSE
% SUM(x-xi)^2/N
rmse_pos = getRMSE(transformedPos,vicon_pos_syn)
rmse_total = sqrt(rmse_pos.x^2 + rmse_pos.y^2 + rmse_pos.z^2)

%% plot
figure(1)
grid on 
hold on
axis([-1 1 -1 1 -0.5 0.5])
% plot3(est_pos_syn_down_offset.x, est_pos_syn_down_offset.y, est_pos_syn_down_offset.z, '-r', 'DisplayName', 'non-rotated')
% plot3(transformedPos.x, transformedPos.y, transformedPos.z, '-g', 'DisplayName', 'rotated')
% plot3(vicon_pos_syn.x, vicon_pos_syn.y, vicon_pos_syn.z, '-k', 'DisplayName', 'true' )

plot3(vicon_pos_syn.x, vicon_pos_syn.y, vicon_pos_syn.z, '-r', 'DisplayName', 'true')
if experiment == 1 ||  experiment == 3
    plot3(transformedPos.x, transformedPos.y, transformedPos.z, '-g', 'DisplayName', 'snapvio UKF')
    
elseif experiment == 2 ||  experiment == 4
    plot3(transformedPos.x, transformedPos.y, transformedPos.z, '-g', 'DisplayName', 'vioqc UKF')
    
end
hold off

xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
legend show

%%
function error = getRMSE(estPos, truePos)
% error.x = (estPos.x-truePos.x).^2;
% error = sum(error.x)
% sizeN = size(truePos.x,1)
% rmseError = sqrt(error/sizeN)

error.x = sqrt(sum((estPos.x-truePos.x).^2)/size(truePos.x,1));
error.y = sqrt(sum((estPos.y-truePos.y).^2)/size(truePos.y,1));
error.z = sqrt(sum((estPos.z-truePos.z).^2)/size(truePos.z,1));
end

%% offset_rotation = getOffsetRot(est_rot_syn_down, vicon_rot_syn); 
% R1 * Rdel = R2
% Rdel = R1' * R2
function offset_rot = getOffsetRot(rotA, rotB)
% est = rotA.rotMat(:,:,1)
% vicon = rotB.rotMat(:,:,1)
offset_rot.rotMat = rotA.rotMat(:,:,1)' * rotB.rotMat(:,:,1);
% offset = offset_rot.rotMat
end

%%
function offset_pos = getOffsetPos(posA, posB)
offset_pos.x = posA.x(1) - posB.x(1);
offset_pos.y = posA.y(1) - posB.y(1);
offset_pos.z = posA.z(1) - posB.z(1);
end

%% est_rot_syn_down_offset = MultRotOffset(est_rot_syn_down, offset_rotation); 
function offset_rot = MultRotOffset(rot, offset)
% offset_rot.rotMat = offset.rotMat .* rot.rotMat;
offset_rot.rotMat = rot.rotMat .* offset.rotMat;
% sizeA = size(rot.rotMat)
% sizeB = size(offset.rotMat)
% sizeC = size(offset_rot.rotMat)
end

%%
function offset_pos = subtractOffset(pos, offset)
offset_pos.x = pos.x - offset.x;
offset_pos.y = pos.y - offset.y;
offset_pos.z = pos.z - offset.z;
end

%%
function [rotm,time] = odom_2_rotm_and_time(odom)

Structs = readMessages(odom,'DataFormat','struct');
quat_x = cellfun(@(m) double(m.Pose.Pose.Orientation.X),Structs);
quat_y = cellfun(@(m) double(m.Pose.Pose.Orientation.Y),Structs);
quat_z = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),Structs);
quat_w = cellfun(@(m) double(m.Pose.Pose.Orientation.W),Structs);
rotm = quat2rotm([quat_w,quat_x,quat_y,quat_z]);
time = cellfun(@(m) double(double(m.Header.Stamp.Sec)+double(m.Header.Stamp.Nsec)*10e-10),Structs);
end

%%
function pos = odom_2_pos(odom)

robot_odomStructs = readMessages(odom,'DataFormat','struct');
x = cellfun(@(m) double(m.Pose.Pose.Position.X),robot_odomStructs);
y = cellfun(@(m) double(m.Pose.Pose.Position.Y),robot_odomStructs);
z = cellfun(@(m) double(m.Pose.Pose.Position.Z),robot_odomStructs);
odom_time = cellfun(@(m) double(double(m.Header.Stamp.Sec)+double(m.Header.Stamp.Nsec)*10e-10),robot_odomStructs);
pos.x = x;
pos.y = y;
pos.z = z;
pos.time = odom_time;

end

%%
function trimmed = trimData(pose, index)
trimmed.x = pose.x(index:end,1);
trimmed.y = pose.y(index:end,1);
trimmed.z = pose.z(index:end,1);
trimmed.time = pose.time(index:end,1);

end

%% vicon_rot_syn = trimDataRot(vicon_rot, vicon_rot_time, 13);

function trimmedRot = trimDataRot(rot, time, index)
trimmedRot.rotMat = rot(:,:,index:end);
trimmedRot.time = time(index:end,1);

end

function truncated = truncate(array, endIndex)
truncated.x = array.x(1:endIndex);
truncated.y = array.y(1:endIndex);
truncated.z = array.z(1:endIndex);
truncated.time = array.time(1:endIndex);
end