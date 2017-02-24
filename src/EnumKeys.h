//
// Created by llach on 23.02.17.
//

#ifndef MEKA_GUIDING_ENUMKEYS_H
#define MEKA_GUIDING_ENUMKEYS_H

enum class tf_key {
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    ROLL,
    PITCH,
    YAW,
    TF_KEY_MAX = YAW
};

enum class cmd_key {
    LINEAR_X,
    LINEAR_Y,
    LINEAR_Z,
    ANGULAR_X,
    ANGULAR_Y,
    ANGUALR_Z,
    CMD_KEY_MAX = ANGUALR_Z
};

enum class dir_key {
    POSITIVE,
    NEGATIVE,
    BIDIRECTIONAL,
    DIR_KEY_MAX = BIDIRECTIONAL
};

#endif //MEKA_GUIDING_ENUMKEYS_H
