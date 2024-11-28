/*
 * @Author: IntronRewrite weijiehe@sdust.edu.com
 * @Date: 2024-11-24 22:11:21
 * @LastEditors: IntronRewrite weijiehe@sdust.edu.com
 * @LastEditTime: 2024-11-28 22:07:17
 * @FilePath: /lw/IndustrialVision/plan2/1.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 * 
 * 
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *            佛祖保佑     永不宕机     永无BUG
 * 
 *        佛曰:  
 *                写字楼里写字间，写字间里程序员；  
 *                程序人员写程序，又拿程序换酒钱。  
 *                酒醒只在网上坐，酒醉还来网下眠；  
 *                酒醉酒醒日复日，网上网下年复年。  
 *                但愿老死电脑间，不愿鞠躬老板前；  
 *                奔驰宝马贵者趣，公交自行程序员。  
 *                别人笑我忒疯癫，我笑自己命太贱；  
 *                不见满街漂亮妹，哪个归得程序员？
 */

/*
 * @Author: IntronRewrite weijiehe@sdust.edu.com
 * @Date: 2024-11-24 22:11:21
 * @LastEditors: IntronRewrite weijiehe@sdust.edu.com
 * @LastEditTime: 2024-11-24 22:42:55
 * @FilePath: /lw/IndustrialVision/plan2/1.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv) {
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 加载点云文件
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("../ref_model.ply", *ref_cloud) == -1) {
        PCL_ERROR("无法读取文件 ref_model.ply \n");
        return (-1);
    }
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("../src_model.ply", *src_cloud) == -1) {
        PCL_ERROR("无法读取文件 src_model.ply \n");
        return (-1);
    }

    // 创建 PCLVisualizer 对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1); // 设置背景颜色为白色

    // 将参考点云添加到查看器
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ref_color(ref_cloud, 255, 0, 0); // 红色
    viewer->addPointCloud<pcl::PointXYZ>(ref_cloud, ref_color, "ref cloud");

    // 将源点云添加到查看器
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_color(src_cloud, 0, 255, 0); // 绿色
    viewer->addPointCloud<pcl::PointXYZ>(src_cloud, src_color, "src cloud");

    // 设置点云属性
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ref cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "src cloud");

    // 添加坐标系
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters(); // 初始化相机参数

    // 主循环
    while (!viewer->wasStopped()) {
        viewer->spinOnce(1000000); // 每次循环等待100毫秒
    }

    return 0;
}