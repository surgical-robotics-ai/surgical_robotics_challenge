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
    // virtual void keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods) override;
    // virtual void mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes) override;
    // virtual void mousePosUpdate(GLFWwindow *a_window, double x_pos, double y_pos) override {}
    // virtual void mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;
    // virtual void reset() override;
    // virtual bool close() override;

    // DrillingPublisher *m_drillingPub;

protected:
    // Subscribe communication loss topic
    void communication_loss_cb(const std_msgs::Bool::ConstPtr& comloss);


    // Initialize tool cursors
    // void toolCursorInit(const afWorldPtr);

    // void incrementDevicePos(cVector3d a_pos);

    // void incrementDeviceRot(cVector3d a_rot);

    // void toolCursorsInitialize();

    // // update position of shaft tool cursors
    // void toolCursorsPosUpdate(cTransform a_devicePose);

    // void resetDrill();

    // // check for shaft collision
    // void checkShaftCollision(void);

    // // update position of drill mesh
    // void drillPoseUpdateFromCursors(void);

    // // toggles size of the drill burr
    // void changeBurrSize(int burrType);

    // bool getOverrideDrillControl() { return m_overrideDrillControl; }

    // void setOverrideDrillControl(bool val) { m_overrideDrillControl = val; }

private:
    bool m_comloss = false;
    ros::NodeHandle* m_rosNode;
    ros::Subscriber m_commLossSub;
    // Commented during merge
    // cTransform T_d; // Drills target pose
    // cTransform T_i; // Input device transform
    // cVector3d V_i;  // Input device linear velocity
    cTransform m_T_d, m_T_d_init; // Drills target pose
    cTransform m_T_i;             // Input device transform
    cVector3d m_V_i;              // Input device linear velocity

    bool m_overrideDrillControl = false;

    cVoxelObject *m_voxelObj;

    cToolCursor *m_targetToolCursor;

    int m_renderingMode = 0;

    double m_opticalDensity;

    cMutex m_mutexVoxel;

    cCollisionAABBBox m_volumeUpdate;

    cColorb m_zeroColor;

    bool m_flagStart = true;

    int m_counter = 0;

    cGenericObject *m_selectedObject = NULL;

    // a haptic device handler
    cHapticDeviceHandler *m_deviceHandler;

    // a pointer to the current haptic device
    cGenericHapticDevicePtr m_hapticDevice;

    bool m_flagMarkVolumeForUpdate = false;

    afRigidBodyPtr m_drillRigidBody;

    afVolumePtr m_volumeObject;

    cShapeSphere *m_burrMesh;

    // tool's rotation matrix
    cMatrix3d m_toolRotMat;

    // rate of drill movement
    double m_drillRate = 0.020f;

    // Local offset between shaft tool cursors
    double m_dX = 0.03;

    // camera to render the world
    afCameraPtr m_mainCamera, m_stereoCameraL, m_stereoCameraR;

    bool m_showDrill = true;

    bool m_showGoalProxySpheres = false;

    // list of tool cursors
    vector<cToolCursor *> m_toolCursorList;

    // radius of tool cursors
    vector<double> m_toolCursorRadius{0.02, 0.013, 0.015, 0.017, 0.019, 0.021, 0.023, 0.025};

    // warning pop-up panel
    cPanel *m_warningPopup;
    cLabel *m_warningText;

    // panel to display current drill size
    cPanel *m_drillSizePanel;
    cLabel *m_drillSizeText;
    cLabel *m_drillControlModeText;

    cPanel *m_distancePanel;//Added
    cLabel *m_distanceText;//Added
    cPanel *m_colorPanel;//Added

    cTransform world_T_voxel;
    cTransform voxel_T_tool;

    // EDT display structures
    // Path to EDT
    // std::string obj_path;

    // Visualisation objects
    // double m_distance_object1 = 0;
    // double m_distance_object2 = 0;
    // double m_distance_object3 = 0;

    // Array3d <float> edtGrid1;
    // Array3d<float> edtGrid2;
    // Array3d<float> edtGrid3;

    // EDT resolution
    int edtres;

    int index_x;
    int index_y;
    int index_z;
    vector<double> force_direction{0, 0, 0};
    cVector3d force_edt;
  
    bool m_flag_force_arrow = false;
    cMesh* arrow_force;

    cAudioSource* m_beepAudioSource = nullptr; //Added for audio
    cAudioBuffer* m_beepAudioBuffer = nullptr; //Added for audio
    cAudioDevice* m_beepAudioDevice = nullptr; //Added for audio

    bool m_flag_sdf = true;

    // current and maximum distance between proxy and goal spheres
    double m_currError = 0;
    double m_maxError = 0;

    // for storing index of follow sphere
    int m_targetToolCursorIdx = 0;

    // toggles whether the drill mesh should move slowly towards the followSphere
    // or make a sudden jump
    bool m_suddenJump = true;

    // index of current drill size
    int m_activeBurrIdx = 0;

    // A map of drill burr indices, radius and description
    map<int, pair<double, string>> m_drillBurrSizes;

    // color property of bone
    cColorb m_boneColor;

    // get color of voxels at (x,y,z)
    cColorb m_storedColor;

    bool m_enableVolumeSmoothing = false;
    int m_volumeSmoothingLevel = 2;

    cLabel *m_volumeSmoothingText;

    cAudioSource *m_drillAudioSource = nullptr;
    cAudioBuffer *m_drillAudioBuffer = nullptr;
    cAudioDevice *m_drillAudioDevice = nullptr;
};

AF_REGISTER_SIMULATOR_PLUGIN(afAutoCompletePlugin)
