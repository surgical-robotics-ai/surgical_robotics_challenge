// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>

using namespace std;
using namespace ambf;

enum HapticStates
{
    HAPTIC_IDLE,
    HAPTIC_SELECTION
};

class afAutoCompletePlugin : public afSimulatorPlugin
{
public:
    afAutoCompletePlugin();
    virtual int init(int argc, char **argv, const afWorldPtr a_afWorld) override;
    virtual void keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods) override;
    // virtual void mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes) override;
    // virtual void mousePosUpdate(GLFWwindow *a_window, double x_pos, double y_pos) override {}
    // virtual void mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;
    // virtual void reset() override;
    // virtual bool close() override;


protected:
    // Subscribe communication loss topic
    void communication_loss_cb(const std_msgs::Bool::ConstPtr& comloss);
    void psm1_recovery_cb(const std_msgs::Bool::ConstPtr& recovery);
    void psm2_recovery_cb(const std_msgs::Bool::ConstPtr& recovery);


private:
    bool m_comloss = false;
    bool m_psm1_recovery = false;
    bool m_psm2_recovery = false;
    bool m_comloss_text = false;
    ros::NodeHandle* m_rosNode;
    ros::Subscriber m_commLossSub;
    ros::Subscriber m_psm1_recoverySub;
    ros::Subscriber m_psm2_recoverySub;

    // camera to render the world
    afCameraPtr m_mainCamera;
    afCameraPtr m_stereoCameraL;
    afCameraPtr m_stereoCameraR;

    afRigidBodyPtr m_PSM1Tool, m_PSM2Tool;
    afRigidBodyPtr m_PSM1pitch, m_PSM2pitch;
    afRigidBodyPtr m_PSM1yaw, m_PSM2yaw;
    afRigidBodyPtr m_PSM1gripper1, m_PSM2gripper1;
    afRigidBodyPtr m_PSM1gripper2, m_PSM2gripper2;
    afRigidBodyPtr m_PSM1_ghost_Tool, m_PSM2_ghost_Tool;
    afRigidBodyPtr m_PSM1_ghost_pitch, m_PSM2_ghost_pitch;
    afRigidBodyPtr m_PSM1_ghost_yaw, m_PSM2_ghost_yaw;
    afRigidBodyPtr m_PSM1_ghost_gripper1, m_PSM2_ghost_gripper1;
    afRigidBodyPtr m_PSM1_ghost_gripper2, m_PSM2_ghost_gripper2;
    afRigidBodyPtr m_PSM1_remote_Tool, m_PSM2_remote_Tool;
    afRigidBodyPtr m_PSM1_remote_pitch, m_PSM2_remote_pitch;
    afRigidBodyPtr m_PSM1_remote_yaw, m_PSM2_remote_yaw;
    afRigidBodyPtr m_PSM1_remote_gripper1, m_PSM2_remote_gripper1;
    afRigidBodyPtr m_PSM1_remote_gripper2, m_PSM2_remote_gripper2;

    //For text
    cLabel *m_comStatus;
    cLabel *m_legend;
    cLabel *m_ori_recovery;

};

AF_REGISTER_SIMULATOR_PLUGIN(afAutoCompletePlugin)
