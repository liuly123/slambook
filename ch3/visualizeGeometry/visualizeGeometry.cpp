#include <iostream>
#include <iomanip>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

#include <pangolin/pangolin.h>

struct RotationMatrix
{
    Matrix3d matrix = Matrix3d::Identity();
};

ostream& operator << ( ostream& out, const RotationMatrix& r )
{
    out.setf(ios::fixed);
    Matrix3d matrix = r.matrix;
    out<<'=';
    out<<"["<<setprecision(2)<<matrix(0,0)<<","<<matrix(0,1)<<","<<matrix(0,2)<<"],"
       << "["<<matrix(1,0)<<","<<matrix(1,1)<<","<<matrix(1,2)<<"],"
       << "["<<matrix(2,0)<<","<<matrix(2,1)<<","<<matrix(2,2)<<"]";
    return out;
}

istream& operator >> (istream& in, RotationMatrix& r )
{
    return in;
}

struct TranslationVector
{
    Vector3d trans = Vector3d(0,0,0);
};

ostream& operator << (ostream& out, const TranslationVector& t)
{
    out<<"=["<<t.trans(0)<<','<<t.trans(1)<<','<<t.trans(2)<<"]";
    return out;
}

istream& operator >> ( istream& in, TranslationVector& t)
{
    return in;
}

struct QuaternionDraw
{
    Quaterniond q;
};

ostream& operator << (ostream& out, const QuaternionDraw quat )
{
    auto c = quat.q.coeffs();
    out<<"=["<<c[0]<<","<<c[1]<<","<<c[2]<<","<<c[3]<<"]";
    return out;
}

istream& operator >> (istream& in, const QuaternionDraw quat)
{
    return in;
}

int main ( int argc, char** argv )
{
    pangolin::CreateWindowAndBind ( "图形显示", 1000, 600 );///窗口
    glEnable ( GL_DEPTH_TEST );
    // 定义一个相机用来显示场景
    pangolin::OpenGlRenderState s_cam (
            pangolin::ProjectionMatrix ( 1000, 600, 420, 420, 500, 300, 0.1, 1000 ),///相机图像的长宽、内参、可见的远近范围
            pangolin::ModelViewLookAt ( 3,3,3,0,0,0,pangolin::AxisY )/// 初始的位姿
    );

    const int UI_WIDTH = 500;///UI的宽度
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -1000.0f/600.0f).SetHandler(&handler);//创建相机视图

    // UI面板
    pangolin::Var<RotationMatrix> rotation_matrix("ui.R", RotationMatrix());/// 旋转矩阵
    pangolin::Var<TranslationVector> translation_vector("ui.t", TranslationVector());/// 平移向量
    pangolin::Var<TranslationVector> euler_angles("ui.rpy", TranslationVector()); /// 旋转的RPY角
    pangolin::Var<QuaternionDraw> quaternion("ui.q", QuaternionDraw()); /// 旋转的四元数
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));//创建UI面板

    // 不断更新界面
    while ( !pangolin::ShouldQuit() )
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );//清除屏幕并激活要渲染的视图
        d_cam.Activate( s_cam );

        pangolin::OpenGlMatrix matrix = s_cam.GetModelViewMatrix();//获取相机视图的位姿
        Matrix<double,4,4> m = matrix;/// 齐次旋转矩阵
        // m = m.inverse();
        RotationMatrix R;
        for (int i=0; i<3; i++)
            for (int j=0; j<3; j++)
                R.matrix(i,j) = m(j,i);
        rotation_matrix = R;

        TranslationVector t;
        t.trans = Vector3d(m(0,3), m(1,3), m(2,3));
        t.trans = -R.matrix*t.trans;
        translation_vector = t;

        TranslationVector euler;
        euler.trans = R.matrix.transpose().eulerAngles(2,1,0);/// transpose转置
        euler_angles = euler;

        QuaternionDraw quat;
        quat.q = Quaterniond(R.matrix);
        quaternion = quat;

        glColor3f(1.0,1.0,1.0);

        pangolin::glDrawColouredCube();//这个Cube是固定的，动的是相机
        // 画出坐标轴
        /// 第一条线
        glLineWidth(3);
        glColor3f ( 0.8f,0.f,0.f );
        glBegin ( GL_LINES );
        glVertex3f( 0,0,0 );
        glVertex3f( 10,0,0 );
        /// 第二条线
        glColor3f( 0.f,0.8f,0.f);
        glVertex3f( 0,0,0 );
        glVertex3f( 0,10,0 );
        /// 第三条线
        glColor3f( 0.2f,0.2f,1.f);
        glVertex3f( 0,0,0 );
        glVertex3f( 0,0,10 );
        glEnd();

        pangolin::FinishFrame();
    }
}
