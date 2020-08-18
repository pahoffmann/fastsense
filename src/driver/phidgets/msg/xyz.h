//
// Created by julian on 8/17/20.
//

#ifndef SRC_XYZ_H
#define SRC_XYZ_H

class XYZ {
public:
    explicit XYZ() = default;
    inline const double& x() const { return data_[0]; }
    inline const double& y() const { return data_[1]; }
    inline const double& z() const { return data_[2]; }

protected:
    double data_[3];
};


#endif //SRC_XYZ_H
