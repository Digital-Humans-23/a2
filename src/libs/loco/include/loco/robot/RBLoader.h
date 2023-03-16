#pragma once

#include <crl-basic/utils/logger.h>
#include <crl-basic/utils/utils.h>

#include "loco/robot/RBEngine.h"
#include "loco/robot/RBUtils.h"

namespace crl::loco {

/* forward declaration */
class RBEngine;

class Robot;

class RBMaterial {
public:
    std::string name;
    V3D color = V3D(0.9, 0.9, 0.9);
};

/**
 * This class is for loading RobotRB, RBJoint and their properties from a
 * various file formats. RBLoader object is constructed when the new robot is
 * created from file and destructed when the loading is finished.
 */
class RBLoader {
public:
    static std::string dataDirectoryPath;

    /* The following lists are only temporal. They contains loaded rbs and
     * joints and once the loading is finished, those rbs and joints are pop
     * from list and moved to rbEngine. Note that some of rbs may not be moved
     * to rbEngine as a result of rb merging. These rb is deleted when RBLoader
     * is destructed */

    // List of links loaded from file.
    std::vector<std::shared_ptr<RB>> rbs;
    // List of joints loaded from file.
    std::vector<std::shared_ptr<RBJoint>> joints;

private:
    std::vector<RBMaterial> materials;

    bool loadVisuals;

public:
    /**
     * Sometimes we only need physical entity of RBs (e.g. when we don't use
     * GUI). In that case, set loadVisuals=false.
     */
    RBLoader(const char *filePath, bool loadVisuals = true);

    ~RBLoader() = default;

    void populateRobot(Robot &robot);

    void populateRBEngine(RBEngine &rbEngine);

private:
    /**
     * This method merge every chidren of rb fixed to rb by fixed joints in
     * recursive fashion.
     */
    void mergeFixedChildren(const std::shared_ptr<RB> &rb);

    /**
     * This method reads a list of rigid bodies from the specified file
     */
    void loadRBsFromFile(const char *fName);

    /**
     * This method returns the reference to the rigid body with the given name,
     * or nullptr if it is not found
     */
    std::shared_ptr<RB> getRBByName(const char *name) const;

    /**
     * This method returns the reference to the joint whose name matches, or
     * nullptr if it is not found
     */
    std::shared_ptr<RBJoint> getJointByName(const char *name) const;

    /**
     * This method loads all the pertinent information regarding the rigid body
     * from a RBS file.
     */
    void loadFromFile(const std::shared_ptr<RB> &rb, FILE *fp) const;

    /**
     * This method is used to load the details of a joint from file.
     */
    void loadFromFile(const std::shared_ptr<RBJoint> &j, FILE *fp) const;

    /**
     * Processes a line of input, if it is specific to this type of joint.
     * Returns true if processed, false otherwise.
     */
    bool processInputLine(const std::shared_ptr<RBJoint> &j, char *line) const;
};

}  // namespace crl::loco
