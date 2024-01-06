import rosbag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

file_name = "/home/ishida/Downloads/pilot1_3.bag"
print("Loading the data from ", file_name)
bag = rosbag.Bag(file_name)

# topics = bag.get_type_and_topic_info()[1].keys()
# print(topics)
# exit(1)

# mtmr_pos = []
# for topic, msg, t in bag.read_messages(topics=['/MTMR/local/measured_cp']):
#     #print(msg.pose.position)
#     mtmr_pos = np.concatenate([mtmr_pos, np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])])


############################################################
# Loading the psm1 and psm2 yawlink state and position
############################################################
psm1_ghost_pos = []
psm1_ghost_vel = []
psm1_ghost_time = []
for topic, msg, t_psm1 in bag.read_messages(topics=['/ambf/env/psm1_ghost/toolyawlink/State']):   
    psm1_ghost_pos = np.concatenate([psm1_ghost_pos, np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])])
    psm1_ghost_vel = np.concatenate([psm1_ghost_vel, np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])])
    psm1_ghost_time.append(t_psm1)

psm2_ghost_pos = []
psm2_ghost_vel = []
psm2_ghost_time = []
for topic, msg, t_psm1 in bag.read_messages(topics=['/ambf/env/psm2_ghost/toolyawlink/State']):   
    psm2_ghost_pos = np.concatenate([psm2_ghost_pos, np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])])
    psm2_ghost_vel = np.concatenate([psm2_ghost_vel, np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])])
    psm2_ghost_time.append(t_psm1)



com_loss = []
com_loss_time = []
for topic, msg, t_loss in bag.read_messages(topics=['/communication_loss']):  
    com_loss.append(msg.data)
    com_loss_time.append(t_loss)

# mtmr_pos = mtmr_pos.reshape([-1,3])
psm1_ghost_pos = psm1_ghost_pos.reshape([-1,3])
psm2_ghost_pos = psm2_ghost_pos.reshape([-1,3])
psm1_ghost_vel = psm1_ghost_vel.reshape([-1,3])
psm2_ghost_vel = psm2_ghost_vel.reshape([-1,3])


# print(psm1_ghost_pos.shape)
# print(psm1_ghost_time[0])
# print(psm1_ghost_time[-1])
# print(len(com_loss))
# print(com_loss_time[0])
# print(com_loss_time[-1])

############################################################
# Loading the psm1(2) com status
############################################################

psm1_status = []
psm2_status = []

j = 0
for i in range(len(psm1_ghost_time)):

    # print(psm1_ghost_time[i])
    # print(com_loss_time[j])

    while(psm1_ghost_time[i] > com_loss_time[j] and  j < len(com_loss_time)-1):
        j+=1
    if com_loss[j-1]:
        psm1_status.append(10)
    else:
        psm1_status.append(0)

j = 0
psm2_loss_vel = []
psm2_live_vel = []

for i in range(len(psm2_ghost_time)):

    # print(psm1_ghost_time[i])
    # print(com_loss_time[j])

    while(psm2_ghost_time[i] > com_loss_time[j] and  j < len(com_loss_time)-1):
        j+=1
    if com_loss[j-1]:
        psm2_status.append(1)
        psm2_loss_vel = np.concatenate([psm2_loss_vel, psm2_ghost_vel[i]])
    else:
        psm2_status.append(0)
        psm2_live_vel = np.concatenate([psm2_live_vel, psm2_ghost_vel[i]])

psm2_loss_vel = psm2_loss_vel.reshape([-1,3])
psm2_live_vel = psm2_live_vel.reshape([-1,3])


#################################################
# Visualize the psm1(2) trajectory
#################################################

visulalize_traj = True
if visulalize_traj:

    # Define the time period that you are visualizing 
    t_start = int(0.6 * len(psm1_status))
    t_end   = int(0.8 * len(psm1_status))


    psm_ghost_pos = np.concatenate([psm1_ghost_pos, psm2_ghost_pos], axis=0)
    psm_status = np.concatenate([psm1_status, psm2_status], axis=0)
    # ax.scatter3D(mtmr_pos[:, 0], mtmr_pos[:, 1], mtmr_pos[:, 2])

    ax = plt.axes(projection="3d")
    # ax = plt.gcf().add_subplot(1, 2, 1, projection="3d")

    sctt = ax.scatter3D(psm1_ghost_pos[t_start:t_end, 0], psm1_ghost_pos[t_start:t_end, 1], psm1_ghost_pos[t_start:t_end, 2], c=psm1_status[t_start:t_end])
    ax.plot([psm1_ghost_pos[t_start,0]],[psm1_ghost_pos[t_start,1]],[psm1_ghost_pos[t_start,2]],"X")
    ax.set_title("psm1")
    # plt.show()

    ax = plt.axes(projection="3d")
    # ax = plt.gcf().add_subplot(1, 2, 1, projection="3d")
    # Position
    sctt1 =ax.scatter3D(psm2_ghost_pos[t_start:t_end, 0], psm2_ghost_pos[t_start:t_end, 1], psm2_ghost_pos[t_start:t_end, 2], c=psm2_status[t_start:t_end])
    ax.plot([psm2_ghost_pos[t_start,0]],[psm2_ghost_pos[t_start,1]],[psm2_ghost_pos[t_start,2]],"X")


    # Velocity
    # sctt1 =ax.scatter3D(psm2_ghost_vel[t_start:t_end, 0], psm2_ghost_vel[t_start:t_end, 1], psm2_ghost_vel[t_start:t_end, 2], c=psm2_status[t_start:t_end])
    # ax.plot([psm1_ghost_vel[t_start,0]],[psm1_ghost_vel[t_start,1]],[psm1_ghost_vel[t_start,2]],"X")

    ax.set_title("psm2")


    # Plot all together
    # sctt =ax.scatter3D(psm_ghost_pos[:, 0], psm_ghost_pos[:, 1], psm_ghost_pos[:, 2], c=psm_status)
    # plt.colorbar(sctt, ax = ax, shrink = 0.5, aspect = 5)

    plt.show()


    # Get plot for the velocity norm
    plt.plot(np.arange(0, len(psm2_status[t_start:t_end])), np.linalg.norm(psm2_ghost_vel[t_start:t_end,:],axis=1))
    plt.plot(np.arange(0, len(psm2_status[t_start:t_end])), psm2_status[t_start:t_end])

    print("Vel average at loss", np.mean(psm2_loss_vel))
    print("Vel average at live",np.mean(psm2_live_vel))


    plt.show()

##################################
# Compare a average velocity
##################################

velocity_flag = False
if(velocity_flag):

    vel_normal = []
    vel_loss = []

    status_ = 0


    i = 0
    while i < (len(psm2_status))-1:
        tmp = []

        while (psm2_status[i] == status_ and i < len(psm2_status)-1):
            tmp.append(np.linalg.norm(psm2_ghost_vel[i]))
            i=i+1
        
        if status_ == 0:
            vel_normal.append(np.mean(tmp))
            status_ = 1
        else:
            vel_loss.append(np.mean(tmp))
            status_ = 0

    print("Normal velocity")
    print(vel_normal)
    print("Loss velocity")
    print(vel_loss)

    plt.plot(np.arange(0, len(vel_normal)), vel_normal)
    plt.plot(np.arange(0, len(vel_loss)), vel_loss)
    plt.legend(["Normal", "loss"])

    plt.show()

    import scipy.stats as stats
    print("Vel average at loss", np.mean(vel_normal))
    print("Vel average at live",np.mean(vel_loss))
    print(stats.ttest_ind(a=vel_normal,b=vel_loss,equal_var=False))

# plt.plot(np.arange(0, len(vel_normal)), np.array(vel_normal)/np.array(vel_loss[0:len(vel_normal)))
# plt.show()