//
// Created by julian on 8/17/20.
//

#ifndef SRC_THREE_DIM_H
#define SRC_THREE_DIM_H

class ThreeDim {
public:
    explicit ThreeDim(const double* data);
    const double& x() const;
    const double& y() const;
    const double& z() const;
private:
    const double data_[3];
};


#endif //SRC_THREE_DIM_H
