/**
BSD 3-Clause License

Copyright (c) 2018, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <memory>

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <ceres/jet.h> 
#include <visnav/common_types.h>

namespace visnav {

template <typename Scalar>
class AbstractCamera;

template <typename Scalar>
class PinholeCamera : public AbstractCamera<Scalar> {
 public:
  static constexpr size_t N = 8;

  typedef Eigen::Matrix<Scalar, 2, 1> Vec2;
  typedef Eigen::Matrix<Scalar, 3, 1> Vec3;

  typedef Eigen::Matrix<Scalar, N, 1> VecN;

  PinholeCamera() = default;
  PinholeCamera(const VecN& p) : param(p) {}

  static PinholeCamera<Scalar> getTestProjections() {
    VecN vec1;
    vec1 << 0.5 * 805, 0.5 * 800, 505, 509, 0, 0, 0, 0;
    PinholeCamera<Scalar> res(vec1);
    return res;
  }

  Scalar* data() { return param.data(); }

  const Scalar* data() const { return param.data(); }

  static std::string getName() { return "pinhole"; }
  std::string name() const { return getName(); }

  virtual Vec2 project(const Vec3& p) const {
    const Scalar& fx = param[0];
    const Scalar& fy = param[1];
    const Scalar& cx = param[2];
    const Scalar& cy = param[3];

    const Scalar& x = p[0];
    const Scalar& y = p[1];
    const Scalar& z = p[2];

    Vec2 res;

// TODO SHEET 2: implement camera model
    // UNUSED(fx);
    // UNUSED(fy);
    // UNUSED(cx);
    // UNUSED(cy);
    // UNUSED(x);
    // UNUSED(y);
    // UNUSED(z);
    res(0) = fx * (x / z) + cx;
    res(1) = fy * (y / z) + cy; 
    
    return res;
  }

  virtual Vec3 unproject(const Vec2& p) const {
    const Scalar& fx = param[0];
    const Scalar& fy = param[1];
    const Scalar& cx = param[2];
    const Scalar& cy = param[3];

    Vec3 res;

// TODO SHEET 2: implement camera model
    // UNUSED(p);
    // UNUSED(fx);
    // UNUSED(fy);
    // UNUSED(cx);
    // UNUSED(cy);
    Scalar one = Scalar(1);
    Scalar mx = (p(0) - cx) / fx;
    Scalar my = (p(1) - cy) / fy;
    Scalar denom = ceres::sqrt(mx * mx + my * my + one);
    res(0) = mx / denom;
    res(1) = my / denom;
    res(2) = one / denom;


    return res;
  }

  const VecN& getParam() const { return param; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  VecN param = VecN::Zero();
};

template <typename Scalar = double>
class ExtendedUnifiedCamera : public AbstractCamera<Scalar> {
 public:
  // NOTE: For convenience for serialization and handling different camera
  // models in ceres functors, we use the same parameter vector size for all of
  // them, even if that means that for some certain entries are unused /
  // constant 0.
  static constexpr int N = 8;

  typedef Eigen::Matrix<Scalar, 2, 1> Vec2;
  typedef Eigen::Matrix<Scalar, 3, 1> Vec3;
  typedef Eigen::Matrix<Scalar, 4, 1> Vec4;

  typedef Eigen::Matrix<Scalar, N, 1> VecN;

  ExtendedUnifiedCamera() = default;
  ExtendedUnifiedCamera(const VecN& p) : param(p) {}

  static ExtendedUnifiedCamera getTestProjections() {
    VecN vec1;
    vec1 << 0.5 * 500, 0.5 * 500, 319.5, 239.5, 0.51231234, 0.9, 0, 0;
    ExtendedUnifiedCamera res(vec1);

    return res;
  }

  Scalar* data() { return param.data(); }
  const Scalar* data() const { return param.data(); }

  static const std::string getName() { return "eucm"; }
  std::string name() const { return getName(); }

  virtual Vec2 project(const Vec3& p) const {
    const Scalar& fx = param[0];
    const Scalar& fy = param[1];
    const Scalar& cx = param[2];
    const Scalar& cy = param[3];
    const Scalar& alpha = param[4];
    const Scalar& beta = param[5];

    const Scalar& x = p[0];
    const Scalar& y = p[1];
    const Scalar& z = p[2];

    Vec2 res;

// TODO SHEET 2: implement camera model
    // UNUSED(fx);
    // UNUSED(fy);
    // UNUSED(cx);
    // UNUSED(cy);
    // UNUSED(alpha);
    // UNUSED(beta);
    // UNUSED(x);
    // UNUSED(y);
    // UNUSED(z);
  Scalar one = Scalar(1);
  Scalar d = ceres::sqrt(beta * (x*x + y*y) + z*z);
  res(0) = fx * (x / (alpha*d+(one-alpha)*z)) + cx;
  res(1) = fy * (y / (alpha*d+(one-alpha)*z)) + cy;

    return res;
  }

  virtual Vec3 unproject(const Vec2& p) const {
    const Scalar& fx = param[0];
    const Scalar& fy = param[1];
    const Scalar& cx = param[2];
    const Scalar& cy = param[3];
    const Scalar& alpha = param[4];
    const Scalar& beta = param[5];

    Vec3 res;

// TODO SHEET 2: implement camera model
    // UNUSED(p);
    // UNUSED(fx);
    // UNUSED(fy);
    // UNUSED(cx);
    // UNUSED(cy);
    // UNUSED(alpha);
    // UNUSED(beta);
  Scalar one = Scalar(1);
  Scalar two = Scalar(2);
  Scalar mx = (p(0)-cx)/fx;
  Scalar my = (p(1)-cy)/fy;
  Scalar r_square = mx*mx + my*my;
  Scalar mz = (one-beta*alpha*alpha*r_square) / 
  ((alpha * ceres::sqrt(one-(two*alpha-one)*beta*r_square))+(one-alpha));
  Scalar denom = ceres::sqrt(mx*mx + my*my + mz*mz);
  res(0)=mx/denom;
  res(1)=my/denom;
  res(2)=mz/denom;


    return res;
  }

  const VecN& getParam() const { return param; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  VecN param = VecN::Zero();
};

template <typename Scalar>
class DoubleSphereCamera : public AbstractCamera<Scalar> {
 public:
  static constexpr size_t N = 8;

  typedef Eigen::Matrix<Scalar, 2, 1> Vec2;
  typedef Eigen::Matrix<Scalar, 3, 1> Vec3;

  typedef Eigen::Matrix<Scalar, N, 1> VecN;

  DoubleSphereCamera() = default;
  DoubleSphereCamera(const VecN& p) : param(p) {}

  static DoubleSphereCamera<Scalar> getTestProjections() {
    VecN vec1;
    vec1 << 0.5 * 805, 0.5 * 800, 505, 509, 0.5 * -0.150694, 0.5 * 1.48785, 0,
        0;
    DoubleSphereCamera<Scalar> res(vec1);

    return res;
  }

  Scalar* data() { return param.data(); }
  const Scalar* data() const { return param.data(); }

  static std::string getName() { return "ds"; }
  std::string name() const { return getName(); }

  virtual Vec2 project(const Vec3& p) const {
    const Scalar& fx = param[0];
    const Scalar& fy = param[1];
    const Scalar& cx = param[2];
    const Scalar& cy = param[3];
    const Scalar& xi = param[4];
    const Scalar& alpha = param[5];

    const Scalar& x = p[0];
    const Scalar& y = p[1];
    const Scalar& z = p[2];

    Vec2 res;

// TODO SHEET 2: implement camera model
    // UNUSED(fx);
    // UNUSED(fy);
    // UNUSED(cx);
    // UNUSED(cy);
    // UNUSED(xi);
    // UNUSED(alpha);
    // UNUSED(x);
    // UNUSED(y);
    // UNUSED(z);
    Scalar one = Scalar(1);
    Scalar d1 = ceres::sqrt(x*x + y*y +z*z);
    Scalar d2 = ceres::sqrt(x*x + y*y + (xi*d1+z)*(xi*d1+z));
    res(0) = fx* (x/(alpha*d2+(one-alpha)*(xi*d1+z)))+cx;
    res(1) = fy* (y/(alpha*d2+(one-alpha)*(xi*d1+z)))+cy;

    return res;
  }

  virtual Vec3 unproject(const Vec2& p) const {
    const Scalar& fx = param[0];
    const Scalar& fy = param[1];
    const Scalar& cx = param[2];
    const Scalar& cy = param[3];
    const Scalar& xi = param[4];
    const Scalar& alpha = param[5];

    Vec3 res;

// TODO SHEET 2: implement camera model
    // UNUSED(p);
    // UNUSED(fx);
    // UNUSED(fy);
    // UNUSED(cx);
    // UNUSED(cy);
    // UNUSED(xi);
    // UNUSED(alpha);
    Scalar one = Scalar(1);
    Scalar two = Scalar(2);
    Scalar mx=(p(0)-cx)/fx;
    Scalar my=(p(1)-cy)/fy;
    Scalar r_square = mx*mx+my*my;
    Scalar mz=(one-alpha*alpha*r_square)/(alpha*ceres::sqrt(one-(two*alpha-one)*r_square)+one-alpha);
    Scalar coff = (mz*xi+ceres::sqrt(mz*mz+(one-xi*xi)*r_square))/(mz*mz+r_square);
    res(0)=coff*mx;
    res(1)=coff*my;
    res(2)=coff*mz-xi;
    return res;
  }

  const VecN& getParam() const { return param; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  VecN param = VecN::Zero();
};

template <typename Scalar = double>
class KannalaBrandt4Camera : public AbstractCamera<Scalar> {
 public:
  static constexpr int N = 8;

  typedef Eigen::Matrix<Scalar, 2, 1> Vec2;
  typedef Eigen::Matrix<Scalar, 3, 1> Vec3;
  typedef Eigen::Matrix<Scalar, 4, 1> Vec4;

  typedef Eigen::Matrix<Scalar, N, 1> VecN;

  KannalaBrandt4Camera() = default;
  KannalaBrandt4Camera(const VecN& p) : param(p) {}

  static KannalaBrandt4Camera getTestProjections() {
    VecN vec1;
    vec1 << 379.045, 379.008, 505.512, 509.969, 0.00693023, -0.0013828,
        -0.000272596, -0.000452646;
    KannalaBrandt4Camera res(vec1);

    return res;
  }

  Scalar* data() { return param.data(); }

  const Scalar* data() const { return param.data(); }

  static std::string getName() { return "kb4"; }
  std::string name() const { return getName(); }

  virtual Vec2 project(const Vec3& p) const {
    const Scalar& fx = param[0];
    const Scalar& fy = param[1];
    const Scalar& cx = param[2];
    const Scalar& cy = param[3];
    const Scalar& k1 = param[4];
    const Scalar& k2 = param[5];
    const Scalar& k3 = param[6];
    const Scalar& k4 = param[7];

    const Scalar& x = p[0];
    const Scalar& y = p[1];
    const Scalar& z = p[2];

    Vec2 res;

// TODO SHEET 2: implement camera model
    // UNUSED(fx);
    // UNUSED(fy);
    // UNUSED(cx);
    // UNUSED(cy);
    // UNUSED(k1);
    // UNUSED(k2);
    // UNUSED(k3);
    // UNUSED(k4);
    // UNUSED(x);
    // UNUSED(y);
    // UNUSED(z);
    Scalar zero = Scalar(0);
    Scalar one = Scalar(1);
    Scalar r = ceres::sqrt(x*x+y*y);
    if (r == zero) {
    res(0) = cx; 
    res(1) = cy;
    return res;
    }
    Scalar theta = ceres::atan2(r,z);
    std::vector<Scalar> coefficients = {k4,zero,k3,zero,k2,zero,k1,zero,one,zero};
    Scalar d_theta = Hornes(coefficients,theta);
    res(0) = (fx*d_theta*x/r) +cx;
    res(1) = (fy*d_theta*y/r) +cy;
    return res;
  }


  virtual Vec3 unproject(const Vec2& p) const {
    const Scalar& fx = param[0];
    const Scalar& fy = param[1];
    const Scalar& cx = param[2];
    const Scalar& cy = param[3];
    const Scalar& k1 = param[4];
    const Scalar& k2 = param[5];
    const Scalar& k3 = param[6];
    const Scalar& k4 = param[7];
    Vec3 res;

// TODO SHEET 2: implement camera model
    // UNUSED(p);
    // UNUSED(fx);
    // UNUSED(fy);
    // UNUSED(cx);
    // UNUSED(cy);
    
    Scalar nine = Scalar(9);
    Scalar seven = Scalar(7);
    Scalar five = Scalar(5);
    Scalar three = Scalar(3);
    Scalar one = Scalar(1);
    Scalar zero = Scalar(0);
    Scalar mx = (p(0)-cx)/fx;
    Scalar my = (p(1)-cy)/fy;
    //std::cout<<mx<<std::endl;
    Scalar ru = ceres::sqrt(mx*mx+my*my);
    std::vector<Scalar> coefficients = {k4, zero, k3, zero, k2, zero, k1, zero, one, zero};
    std::vector<Scalar> coefficients_d = {nine*k4 ,zero ,seven*k3 ,zero ,five*k2 ,zero ,three*k1 ,zero ,one};
    Scalar theta_star = Newton(ru,coefficients,coefficients_d);
    if(theta_star == zero){
    res(0) = zero; 
    res(1) = zero;
    res(2) = one;
    return res;}
    res(0) = ceres::sin(theta_star)*mx/ru;
    res(1) = ceres::sin(theta_star)*my/ru;
    res(2) = ceres::cos(theta_star);
  
    return res;
  }

  Scalar Hornes(const std::vector<Scalar>& v, Scalar x) const{
    Scalar d_theta =Scalar(0);
    for(size_t i=0;i<=v.size()-1;i++){
        d_theta = d_theta*x + v[i];
    }
    return d_theta;
  }

  Scalar Newton(Scalar x, const std::vector<Scalar>& v, const std::vector<Scalar>& v_d)const{
    Scalar init_guess = Scalar(0.785); // pi/4
    Scalar root = init_guess;
    Scalar prev_x = root;
    for (size_t i = 0; i < 10; i++) {
    Scalar f = Hornes(v, root) - x;
    Scalar f_d = Hornes(v_d, root);
    std::cout<<f_d<<std::endl;
    if (abs(f_d) < 1e-8) {
        break;
    }
    root = root - f / f_d;
    if (abs(root - prev_x) < 1e-6) {  //convergence checking
        break;
    }
    prev_x = root;
    }
    return root;
    
  }

  const VecN& getParam() const { return param; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  VecN param = VecN::Zero();
};

template <typename Scalar>
class AbstractCamera {
 public:
  static constexpr size_t N = 8;

  typedef Eigen::Matrix<Scalar, 2, 1> Vec2;
  typedef Eigen::Matrix<Scalar, 3, 1> Vec3;

  typedef Eigen::Matrix<Scalar, N, 1> VecN;

  virtual ~AbstractCamera() = default;

  virtual Scalar* data() = 0;

  virtual const Scalar* data() const = 0;

  virtual Vec2 project(const Vec3& p) const = 0;

  virtual Vec3 unproject(const Vec2& p) const = 0;

  virtual std::string name() const = 0;

  virtual const VecN& getParam() const = 0;

  inline int width() const { return width_; }
  inline int& width() { return width_; }
  inline int height() const { return height_; }
  inline int& height() { return height_; }
  static std::shared_ptr<AbstractCamera> from_data(const std::string& name,
                                                   const Scalar* sIntr) {
    if (name == DoubleSphereCamera<Scalar>::getName()) {
      Eigen::Map<Eigen::Matrix<Scalar, 8, 1> const> intr(sIntr);
      return std::shared_ptr<AbstractCamera>(
          new DoubleSphereCamera<Scalar>(intr));
    } else if (name == PinholeCamera<Scalar>::getName()) {
      Eigen::Map<Eigen::Matrix<Scalar, 8, 1> const> intr(sIntr);
      return std::shared_ptr<AbstractCamera>(new PinholeCamera<Scalar>(intr));
    } else if (name == KannalaBrandt4Camera<Scalar>::getName()) {
      Eigen::Map<Eigen::Matrix<Scalar, 8, 1> const> intr(sIntr);
      return std::shared_ptr<AbstractCamera>(
          new KannalaBrandt4Camera<Scalar>(intr));
    } else if (name == ExtendedUnifiedCamera<Scalar>::getName()) {
      Eigen::Map<Eigen::Matrix<Scalar, 8, 1> const> intr(sIntr);
      return std::shared_ptr<AbstractCamera>(
          new ExtendedUnifiedCamera<Scalar>(intr));
    } else {
      std::cerr << "Camera model " << name << " is not implemented."
                << std::endl;
      std::abort();
    }
  }

  // Loading from double sphere initialization
  static std::shared_ptr<AbstractCamera> initialize(const std::string& name,
                                                    const Scalar* sIntr) {
    Eigen::Matrix<Scalar, 8, 1> init_intr;

    if (name == DoubleSphereCamera<Scalar>::getName()) {
      Eigen::Map<Eigen::Matrix<Scalar, 8, 1> const> intr(sIntr); //jiang zhege leixing fengzhuang qilai 

      init_intr = intr;

      return std::shared_ptr<AbstractCamera>(
          new DoubleSphereCamera<Scalar>(init_intr));
    } else if (name == PinholeCamera<Scalar>::getName()) {
      Eigen::Map<Eigen::Matrix<Scalar, 8, 1> const> intr(sIntr);

      init_intr = intr;
      init_intr.template tail<4>().setZero();

      return std::shared_ptr<AbstractCamera>(
          new PinholeCamera<Scalar>(init_intr));
    } else if (name == KannalaBrandt4Camera<Scalar>::getName()) {
      Eigen::Map<Eigen::Matrix<Scalar, 8, 1> const> intr(sIntr);

      init_intr = intr;
      init_intr.template tail<4>().setZero();

      return std::shared_ptr<AbstractCamera>(
          new KannalaBrandt4Camera<Scalar>(init_intr));
    } else if (name == ExtendedUnifiedCamera<Scalar>::getName()) {
      Eigen::Map<Eigen::Matrix<Scalar, 8, 1> const> intr(sIntr);

      init_intr = intr;
      init_intr.template tail<4>().setZero();
      init_intr[4] = 0.5;
      init_intr[5] = 1;

      return std::shared_ptr<AbstractCamera>(
          new ExtendedUnifiedCamera<Scalar>(init_intr));
    } else {
      std::cerr << "Camera model " << name << " is not implemented."
                << std::endl;
      std::abort();
    }
  }

 private:
  // image dimensions
  int width_ = 0;
  int height_ = 0;
};

}  // namespace visnav


