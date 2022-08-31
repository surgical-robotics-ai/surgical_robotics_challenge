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


private:
    bool m_comloss = false;
    ros::NodeHandle* m_rosNode;
    ros::Subscriber m_commLossSub;

    // camera to render the world
    afCameraPtr m_mainCamera;
    afCameraPtr m_stereoCameraL;
    afCameraPtr m_stereoCameraR;

    //For text
    cLabel *m_comStatus;
    cLabel *m_legend;

};

AF_REGISTER_SIMULATOR_PLUGIN(afAutoCompletePlugin)
