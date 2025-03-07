// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "engine/engine_sensor.h"

#include <stddef.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include "engine/engine_callback.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_io.h"
#include "engine/engine_plugin.h"
#include "engine/engine_ray.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"



//-------------------------------- utility ---------------------------------------------------------

// apply cutoff after each stage
static void apply_cutoff(const mjModel* m, mjData* d, mjtStage stage) {
  // process sensors matching stage and having positive cutoff
  for (int i=0; i < m->nsensor; i++) {
    if (m->sensor_needstage[i] == stage && m->sensor_cutoff[i] > 0) {
      // skip fromto sensors
      if (m->sensor_type[i] == mjSENS_GEOMFROMTO) {
        continue;
      }

      // get sensor info
      int adr = m->sensor_adr[i];
      int dim = m->sensor_dim[i];
      mjtNum cutoff = m->sensor_cutoff[i];

      // process all dimensions
      for (int j=0; j < dim; j++) {
        // real: apply on both sides
        if (m->sensor_datatype[i] == mjDATATYPE_REAL) {
          d->sensordata[adr+j] = mju_clip(d->sensordata[adr+j], -cutoff, cutoff);
        }

        // positive: apply on positive side only
        else if (m->sensor_datatype[i] == mjDATATYPE_POSITIVE) {
          d->sensordata[adr+j] = mju_min(cutoff, d->sensordata[adr+j]);
        }
      }
    }
  }
}



// get xpos and xmat pointers to an object in mjData
static void get_xpos_xmat(const mjData* d, mjtObj type, int id, int sensor_id,
                          mjtNum **xpos, mjtNum **xmat) {
  switch (type) {
  case mjOBJ_XBODY:
    *xpos = d->xpos + 3*id;
    *xmat = d->xmat + 9*id;
    break;
  case mjOBJ_BODY:
    *xpos = d->xipos + 3*id;
    *xmat = d->ximat + 9*id;
    break;
  case mjOBJ_GEOM:
    *xpos = d->geom_xpos + 3*id;
    *xmat = d->geom_xmat + 9*id;
    break;
  case mjOBJ_SITE:
    *xpos = d->site_xpos + 3*id;
    *xmat = d->site_xmat + 9*id;
    break;
  case mjOBJ_CAMERA:
    *xpos = d->cam_xpos + 3*id;
    *xmat = d->cam_xmat + 9*id;
    break;
  default:
    mjERROR("invalid object type in sensor %d", sensor_id);
  }
}

// get global quaternion of an object in mjData
static void get_xquat(const mjModel* m, const mjData* d, mjtObj type, int id, int sensor_id,
                      mjtNum *quat) {
  switch (type) {
  case mjOBJ_XBODY:
    mju_copy4(quat, d->xquat+4*id);
    break;
  case mjOBJ_BODY:
    mju_mulQuat(quat, d->xquat+4*id, m->body_iquat+4*id);
    break;
  case mjOBJ_GEOM:
    mju_mulQuat(quat, d->xquat+4*m->geom_bodyid[id], m->geom_quat+4*id);
    break;
  case mjOBJ_SITE:
    mju_mulQuat(quat, d->xquat+4*m->site_bodyid[id], m->site_quat+4*id);
    break;
  case mjOBJ_CAMERA:
    mju_mulQuat(quat, d->xquat+4*m->cam_bodyid[id], m->cam_quat+4*id);
    break;
  default:
    mjERROR("invalid object type in sensor %d", sensor_id);
  }
}


static void cam_project(mjtNum sensordata[2], const mjtNum target_xpos[3],
                        const mjtNum cam_xpos[3], const mjtNum cam_xmat[9],
                        const int cam_res[2], mjtNum cam_fovy,
                        const float cam_intrinsic[4], const float cam_sensorsize[2]) {
  mjtNum fx, fy;

  // translation matrix (4x4)
  mjtNum translation[4][4] = {0};
  translation[0][0] = 1;
  translation[1][1] = 1;
  translation[2][2] = 1;
  translation[3][3] = 1;
  translation[0][3] = -cam_xpos[0];
  translation[1][3] = -cam_xpos[1];
  translation[2][3] = -cam_xpos[2];

  // rotation matrix (4x4)
  mjtNum rotation[4][4] = {0};
  rotation[0][0] = 1;
  rotation[1][1] = 1;
  rotation[2][2] = 1;
  rotation[3][3] = 1;
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      rotation[i][j] = cam_xmat[j*3+i];
    }
  }

  // focal transformation matrix (3x4)
  if (cam_sensorsize[0] && cam_sensorsize[1]) {
    fx = cam_intrinsic[0] / cam_sensorsize[0] * cam_res[0];
    fy = cam_intrinsic[1] / cam_sensorsize[1] * cam_res[1];
  } else {
    fx = fy = .5 / mju_tan(cam_fovy * mjPI / 360.) * cam_res[1];
  }

  mjtNum focal[3][4] = {0};
  focal[0][0] = -fx;
  focal[1][1] =  fy;
  focal[2][2] = 1.0;

  // image matrix (3x3)
  mjtNum image[3][3] = {0};
  image[0][0] = 1;
  image[1][1] = 1;
  image[2][2] = 1;
  image[0][2] = (mjtNum)cam_res[0] / 2.0;
  image[1][2] = (mjtNum)cam_res[1] / 2.0;

  // projection matrix (3x4): product of all 4 matrices
  mjtNum proj[3][4] = {0};
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      for (int k=0; k < 4; k++) {
        for (int l=0; l < 4; l++) {
          for (int n=0; n < 4; n++) {
            proj[i][n] += image[i][j] * focal[j][k] * rotation[k][l] * translation[l][n];
          }
        }
      }
    }
  }

  // projection matrix multiplies homogenous [x, y, z, 1] vectors
  mjtNum pos_hom[4] = {0, 0, 0, 1};
  mju_copy3(pos_hom, target_xpos);

  // project world coordinates into pixel space, see:
  // https://en.wikipedia.org/wiki/3D_projection#Mathematical_formula
  mjtNum pixel_coord_hom[3] = {0};
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 4; j++) {
      pixel_coord_hom[i] += proj[i][j] * pos_hom[j];
    }
  }

  // avoid dividing by tiny numbers
  mjtNum denom = pixel_coord_hom[2];
  if (mju_abs(denom) < mjMINVAL) {
    if (denom < 0) {
      denom = mju_min(denom, -mjMINVAL);
    } else {
      denom = mju_max(denom, mjMINVAL);
    }
  }

  // compute projection
  sensordata[0] = pixel_coord_hom[0] / denom;
  sensordata[1] = pixel_coord_hom[1] / denom;
}



//-------------------------------- sensor ----------------------------------------------------------

// 传感器位置处理函数，负责计算在位置阶段需要更新的传感器数据
void mj_sensorPos(const mjModel* m, mjData* d) {
    // 获取模型约束和传感器相关参数
    int ne = d->ne, nf = d->nf, nefc = d->nefc, nsensor = m->nsensor;
    int nusersensor = 0;  // 用户自定义传感器计数器

    // 检查传感器功能是否被禁用，若禁用则直接返回
    if (mjDISABLED(mjDSBL_SENSOR)) {
        return;
    }

    // 遍历所有传感器，处理处于位置阶段的传感器
    for (int i = 0; i < nsensor; i++) {
        mjtSensor type = (mjtSensor)m->sensor_type[i];

        // 跳过插件类型传感器（插件传感器后续处理）
        if (type == mjSENS_PLUGIN) {
            continue;
        }

        // 仅处理需要位置阶段的传感器
        if (m->sensor_needstage[i] == mjSTAGE_POS) {
            // 获取传感器相关参数
            int objtype = m->sensor_objtype[i];  // 传感器目标类型
            int objid = m->sensor_objid[i];     // 传感器目标ID
            int refid = m->sensor_refid[i];     // 参考对象ID
            int reftype = m->sensor_reftype[i]; // 参考对象类型
            int adr = m->sensor_adr[i];         // 传感器数据存储地址

            mjtNum rvec[3], * xpos, * xmat, * xpos_ref, * xmat_ref;  // 临时存储空间

            // 根据传感器类型进行相应处理
            switch (type) {
            case mjSENS_MAGNETOMETER:                           // 磁力计
                // 将全局磁场矢量转换到站点坐标系
                mju_mulMatTVec(d->sensordata + adr, d->site_xmat + 9 * objid, m->opt.magnetic, 3, 3);
                break;

            case mjSENS_CAMPROJECTION:                          // 相机投影
                // 执行相机投影计算
                cam_project(d->sensordata + adr, d->site_xpos + 3 * objid, d->cam_xpos + 3 * refid,
                    d->cam_xmat + 9 * refid, m->cam_resolution + 2 * refid, m->cam_fovy[refid],
                    m->cam_intrinsic + 4 * refid, m->cam_sensorsize + 2 * refid);
                break;

            case mjSENS_RANGEFINDER:                            // 测距仪
                // 获取站点坐标系Z轴方向
                rvec[0] = d->site_xmat[9 * objid + 2];
                rvec[1] = d->site_xmat[9 * objid + 5];
                rvec[2] = d->site_xmat[9 * objid + 8];
                // 执行射线检测计算距离
                d->sensordata[adr] = mj_ray(m, d, d->site_xpos + 3 * objid, rvec, NULL, 1,
                    m->site_bodyid[objid], NULL);
                break;

            case mjSENS_JOINTPOS:                               // 关节位置
                d->sensordata[adr] = d->qpos[m->jnt_qposadr[objid]];
                break;

            case mjSENS_TENDONPOS:                              // 肌腱长度
                d->sensordata[adr] = d->ten_length[objid];
                break;

            case mjSENS_ACTUATORPOS:                            // 执行器位置
                d->sensordata[adr] = d->actuator_length[objid];
                break;

            case mjSENS_BALLQUAT:                               // 球铰四元数
                mju_copy4(d->sensordata + adr, d->qpos + m->jnt_qposadr[objid]);
                mju_normalize4(d->sensordata + adr);  // 四元数归一化
                break;

            case mjSENS_JOINTLIMITPOS:                         // 关节限位位置
                d->sensordata[adr] = 0;
                // 在约束数组中查找对应关节的限位约束
                for (int j = ne + nf; j < nefc; j++) {
                    if (d->efc_type[j] == mjCNSTR_LIMIT_JOINT && d->efc_id[j] == objid) {
                        d->sensordata[adr] = d->efc_pos[j] - d->efc_margin[j];
                        break;
                    }
                }
                break;

            case mjSENS_TENDONLIMITPOS:                        // 肌腱限位位置
                d->sensordata[adr] = 0;
                // 在约束数组中查找对应肌腱的限位约束
                for (int j = ne + nf; j < nefc; j++) {
                    if (d->efc_type[j] == mjCNSTR_LIMIT_TENDON && d->efc_id[j] == objid) {
                        d->sensordata[adr] = d->efc_pos[j] - d->efc_margin[j];
                        break;
                    }
                }
                break;

            case mjSENS_FRAMEPOS:                               // 坐标系位置
            case mjSENS_FRAMEXAXIS:                             // 坐标系X轴
            case mjSENS_FRAMEYAXIS:                             // 坐标系Y轴
            case mjSENS_FRAMEZAXIS:                             // 坐标系Z轴
                // 获取目标对象的空间位置和旋转矩阵
                get_xpos_xmat(d, objtype, objid, i, &xpos, &xmat);

                // 无参考帧时使用全局坐标系
                if (refid == -1) {
                    if (type == mjSENS_FRAMEPOS) {
                        mju_copy3(d->sensordata + adr, xpos);
                    }
                    else {
                        // 根据类型计算对应轴分量（X/Y/Z轴对应偏移0/1/2）
                        int offset = type - mjSENS_FRAMEXAXIS;
                        d->sensordata[adr] = xmat[offset];
                        d->sensordata[adr + 1] = xmat[offset + 3];
                        d->sensordata[adr + 2] = xmat[offset + 6];
                    }
                }

                // 存在参考帧的情况
                else {
                    get_xpos_xmat(d, reftype, refid, i, &xpos_ref, &xmat_ref);
                    if (type == mjSENS_FRAMEPOS) {
                        mju_sub3(rvec, xpos, xpos_ref);  // 计算相对位置
                        mju_mulMatTVec3(d->sensordata + adr, xmat_ref, rvec);
                    }
                    else {
                        // 计算参考坐标系中的轴方向
                        int offset = type - mjSENS_FRAMEXAXIS;
                        mjtNum axis[3] = { xmat[offset], xmat[offset + 3], xmat[offset + 6] };
                        mju_mulMatTVec3(d->sensordata + adr, xmat_ref, axis);
                    }
                }
                break;

            case mjSENS_FRAMEQUAT:                              // 坐标系四元数
            {
                // 获取目标对象的全局四元数
                mjtNum objquat[4];
                get_xquat(m, d, objtype, objid, i, objquat);

                // 无参考帧时直接复制四元数
                if (refid == -1) {
                    mju_copy4(d->sensordata + adr, objquat);
                }
                else {
                    // 获取参考帧的全局四元数
                    mjtNum refquat[4];
                    get_xquat(m, d, reftype, refid, i, refquat);

                    // 计算相对四元数（参考系到对象的旋转）
                    mju_negQuat(refquat, refquat);
                    mju_mulQuat(d->sensordata + adr, refquat, objquat);
                }
            }
            break;

            case mjSENS_SUBTREECOM:                             // 子树质心位置
                mju_copy3(d->sensordata + adr, d->subtree_com + 3 * objid);
                break;

            case mjSENS_GEOMDIST:                               // 几何体间最短距离
            case mjSENS_GEOMNORMAL:                             // 几何体间最短法线方向
            case mjSENS_GEOMFROMTO:                             // 几何体间最短线段
            {
                // 使用传感器截止值作为碰撞边距
                mjtNum margin = m->sensor_cutoff[i];

                // 初始化输出变量
                mjtNum dist = margin;     // 碰撞距离
                mjtNum fromto[6] = { 0 };   // 连接线段坐标

                // 获取要检测的几何体集合
                int n1, id1;
                if (objtype == mjOBJ_BODY) {  // 如果是物体则获取其所有几何体
                    n1 = m->body_geomnum[objid];
                    id1 = m->body_geomadr[objid];
                }
                else {  // 否则直接使用单个几何体
                    n1 = 1;
                    id1 = objid;
                }
                int n2, id2;
                if (reftype == mjOBJ_BODY) {
                    n2 = m->body_geomnum[refid];
                    id2 = m->body_geomadr[refid];
                }
                else {
                    n2 = 1;
                    id2 = refid;
                }

                // 遍历所有几何体对，寻找最小距离
                for (int geom1 = id1; geom1 < id1 + n1; geom1++) {
                    for (int geom2 = id2; geom2 < id2 + n2; geom2++) {
                        mjtNum fromto_new[6] = { 0 };
                        mjtNum dist_new = mj_geomDistance(m, d, geom1, geom2, margin, fromto_new);
                        if (dist_new < dist) {  // 记录最小距离
                            dist = dist_new;
                            mju_copy(fromto, fromto_new, 6);
                        }
                    }
                }

                // 处理具有相同参数的后续传感器（批量处理优化）
                int write_sensor = 1;
                while (write_sensor) {
                    // 根据类型写入传感器数据
                    if (type == mjSENS_GEOMDIST) {  // 距离值
                        d->sensordata[adr] = dist;
                    }
                    else if (type == mjSENS_GEOMNORMAL) {  // 法线方向
                        mjtNum normal[3] = { fromto[3] - fromto[0], fromto[4] - fromto[1], fromto[5] - fromto[2] };
                        if (normal[0] || normal[1] || normal[2]) {
                            mju_normalize3(normal);  // 归一化法向量
                        }
                        mju_copy3(d->sensordata + adr, normal);
                    }
                    else {  // 线段端点坐标
                        mju_copy(d->sensordata + adr, fromto, 6);
                    }

                    // 检查后续传感器是否具有相同配置
                    if (i + 1 == nsensor) break;

                    // 获取下一个传感器的类型
                    mjtSensor type_next = m->sensor_type[i + 1];

                    // 判断是否匹配参数配置
                    write_sensor = (type_next == mjSENS_GEOMDIST ||
                        type_next == mjSENS_GEOMNORMAL ||
                        type_next == mjSENS_GEOMFROMTO) &&
                        m->sensor_objtype[i + 1] == objtype &&
                        m->sensor_objid[i + 1] == objid &&
                        m->sensor_reftype[i + 1] == reftype &&
                        m->sensor_refid[i + 1] == refid &&
                        m->sensor_cutoff[i + 1] == margin;

                    // 如果匹配则处理后续传感器
                    if (write_sensor) {
                        i++;  // 递增主循环变量
                        adr = m->sensor_adr[i];  // 更新存储地址
                        type = type_next;        // 更新传感器类型
                    }
                }
            }
            break;

            case mjSENS_E_POTENTIAL:                            // 势能
                mj_energyPos(m, d);  // 计算势能
                d->sensordata[adr] = d->energy[0];
                break;

            case mjSENS_E_KINETIC:                              // 动能
                mj_energyVel(m, d);  // 计算动能
                d->sensordata[adr] = d->energy[1];
                break;

            case mjSENS_CLOCK:                                  // 仿真时钟
                d->sensordata[adr] = d->time;
                break;

            case mjSENS_USER:                                   // 用户自定义传感器
                nusersensor++;  // 计数，实际处理需用户实现
                break;

            default:  // 无效传感器类型处理
                mjERROR("位置阶段发现无效传感器类型，传感器编号 %d", i);
            }
        }
    }

    // 处理用户自定义传感器（如果存在且注册了回调函数）
    if (nusersensor && mjcb_sensor) {
        mjcb_sensor(m, d, mjSTAGE_POS);  // 调用用户自定义传感器回调函数处理位置阶段数据
    }

    // 计算插件类型传感器的数值
    if (m->nplugin) {  // 当模型包含插件时执行
        const int nslot = mjp_pluginCount();  // 获取系统插件槽位总数
        // 遍历所有插件实例
        for (int i = 0; i < m->nplugin; i++) {
            const int slot = m->plugin[i];  // 获取当前插件在槽位中的索引
            // 安全获取插件指针（不进行边界检查）
            const mjpPlugin* plugin = mjp_getPluginAtSlotUnsafe(slot, nslot);
            // 插件有效性检查
            if (!plugin) {
                mjERROR("无效插件槽位: %d", slot);  // 抛出错误：插件槽位不存在
            }
            // 检查插件能力标志和阶段需求
            if ((plugin->capabilityflags & mjPLUGIN_SENSOR) &&  // 具有传感器处理能力
                (plugin->needstage == mjSTAGE_POS || plugin->needstage == mjSTAGE_NONE)) {  // 需要位置阶段处理
                // 检查计算函数指针有效性
                if (!plugin->compute) {
                    mjERROR("插件槽位 %d 的计算函数指针为空", slot);  // 抛出错误：无效函数指针
                }
                // 调用插件的计算函数处理传感器数据
                plugin->compute(m, d, i, mjPLUGIN_SENSOR);
            }
        }
    }

    // 对位置阶段的传感器数据应用截止值处理
    apply_cutoff(m, d, mjSTAGE_POS);  // 根据传感器配置的截止范围限制数据值域
}




// 速度相关传感器
void mj_sensorVel(const mjModel* m, mjData* d) {
    int objtype, objid, reftype, refid, adr, nusersensor = 0;
    int ne = d->ne, nf = d->nf, nefc = d->nefc;
    mjtNum xvel[6];

    // 传感器被禁用：直接返回
    if (mjDISABLED(mjDSBL_SENSOR)) {
        return;
    }

    // 处理匹配当前阶段（速度阶段）的传感器
    int subtreeVel = 0;
    for (int i = 0; i < m->nsensor; i++) {
        // 跳过插件传感器 - 这些将在内置传感器类型之后处理
        if (m->sensor_type[i] == mjSENS_PLUGIN) {
            continue;
        }

        if (m->sensor_needstage[i] == mjSTAGE_VEL) {
            // 获取传感器信息
            mjtSensor type = m->sensor_type[i];
            objtype = m->sensor_objtype[i];
            objid = m->sensor_objid[i];
            refid = m->sensor_refid[i];
            reftype = m->sensor_reftype[i];
            adr = m->sensor_adr[i];

            // 当首次遇到需要子树速度的传感器时调用 mj_subtreeVel
            if (subtreeVel == 0 &&
                (type == mjSENS_SUBTREELINVEL ||
                    type == mjSENS_SUBTREEANGMOM ||
                    type == mjSENS_USER)) {
                // 计算 subtree_linvel（子树线速度）和 subtree_angmom（子树角动量）
                mj_subtreeVel(m, d);

                // 标记已计算
                subtreeVel = 1;
            }

            // 根据传感器类型进行处理
            switch (type) {
            case mjSENS_VELOCIMETER:                            // 线速度计
                // xvel = 站点速度，在站点坐标系中
                mj_objectVelocity(m, d, mjOBJ_SITE, objid, xvel, 1);

                // 赋值线速度
                mju_copy3(d->sensordata + adr, xvel + 3);
                break;

            case mjSENS_GYRO:                                   // 陀螺仪
                // xvel = 站点速度，在站点坐标系中
                mj_objectVelocity(m, d, mjOBJ_SITE, objid, xvel, 1);

                // 赋值角速度
                mju_copy3(d->sensordata + adr, xvel);
                break;

            case mjSENS_JOINTVEL:                               // 关节速度
                d->sensordata[adr] = d->qvel[m->jnt_dofadr[objid]];
                break;

            case mjSENS_TENDONVEL:                              // 腱速度
                d->sensordata[adr] = d->ten_velocity[objid];
                break;

            case mjSENS_ACTUATORVEL:                            // 执行器速度
                d->sensordata[adr] = d->actuator_velocity[objid];
                break;

            case mjSENS_BALLANGVEL:                             // 球铰角速度
                mju_copy3(d->sensordata + adr, d->qvel + m->jnt_dofadr[objid]);
                break;

            case mjSENS_JOINTLIMITVEL:                          // 关节限制速度
                d->sensordata[adr] = 0;
                for (int j = ne + nf; j < nefc; j++) {
                    if (d->efc_type[j] == mjCNSTR_LIMIT_JOINT && d->efc_id[j] == objid) {
                        d->sensordata[adr] = d->efc_vel[j];
                        break;
                    }
                }
                break;

            case mjSENS_TENDONLIMITVEL:                         // 腱限制速度
                d->sensordata[adr] = 0;
                for (int j = ne + nf; j < nefc; j++) {
                    if (d->efc_type[j] == mjCNSTR_LIMIT_TENDON && d->efc_id[j] == objid) {
                        d->sensordata[adr] = d->efc_vel[j];
                        break;
                    }
                }
                break;

            case mjSENS_FRAMELINVEL:                            // 框架线速度
            case mjSENS_FRAMEANGVEL:                            // 框架角速度
                // xvel = 物体的6D速度（全局坐标系）
                mj_objectVelocity(m, d, objtype, objid, xvel, 0);

                if (refid > -1) {  // 指定了参考框架
                    mjtNum* xpos, * xmat, * xpos_ref, * xmat_ref, xvel_ref[6], rel_vel[6], cross[3], rvec[3];

                    // 全局坐标系中的物体和参考位置、参考姿态及速度
                    get_xpos_xmat(d, objtype, objid, i, &xpos, &xmat);
                    get_xpos_xmat(d, reftype, refid, i, &xpos_ref, &xmat_ref);
                    mj_objectVelocity(m, d, reftype, refid, xvel_ref, 0);

                    // 计算相对速度
                    mju_sub(rel_vel, xvel, xvel_ref, 6);

                    // 线速度：添加由于参考系旋转引起的修正
                    mju_sub3(rvec, xpos, xpos_ref);
                    mju_cross(cross, rvec, xvel_ref);
                    mju_addTo3(rel_vel + 3, cross);

                    // 投影到参考坐标系
                    mju_mulMatTVec3(xvel, xmat_ref, rel_vel);
                    mju_mulMatTVec3(xvel + 3, xmat_ref, rel_vel + 3);
                }

                // 复制线速度或角速度分量
                if (m->sensor_type[i] == mjSENS_FRAMELINVEL) {
                    mju_copy3(d->sensordata + adr, xvel + 3);
                }
                else {
                    mju_copy3(d->sensordata + adr, xvel);
                }
                break;

            case mjSENS_SUBTREELINVEL:                          // 子树线速度
                mju_copy3(d->sensordata + adr, d->subtree_linvel + 3 * objid);
                break;

            case mjSENS_SUBTREEANGMOM:                          // 子树角动量
                mju_copy3(d->sensordata + adr, d->subtree_angmom + 3 * objid);
                break;

            case mjSENS_USER:                                   // 用户自定义传感器
                nusersensor++;
                break;

            default:
                mjERROR("VEL阶段出现无效类型，传感器 %d", i);
            }
        }
    }

    // 如果检测到用户传感器则填充数据
    if (nusersensor && mjcb_sensor) {
        mjcb_sensor(m, d, mjSTAGE_VEL);
    }

    // 触发插件的计算
    if (m->nplugin) {
        const int nslot = mjp_pluginCount();
        for (int i = 0; i < m->nplugin; i++) {
            const int slot = m->plugin[i];
            const mjpPlugin* plugin = mjp_getPluginAtSlotUnsafe(slot, nslot);
            if (!plugin) {
                mjERROR("无效插件槽位: %d", slot);
            }
            if ((plugin->capabilityflags & mjPLUGIN_SENSOR) && plugin->needstage == mjSTAGE_VEL) {
                if (!plugin->compute) {
                    mjERROR("槽位 %d 的插件 `compute` 函数为空", slot);
                }
                if (subtreeVel == 0) {
                    // 计算 subtree_linvel 和 subtree_angmom
                    mj_subtreeVel(m, d);

                    // 标记已计算
                    subtreeVel = 1;
                }
                plugin->compute(m, d, i, mjPLUGIN_SENSOR);
            }
        }
    }

    // 应用截止频率
    apply_cutoff(m, d, mjSTAGE_VEL);
}

// acceleration/force-dependent sensors
void mj_sensorAcc(const mjModel* m, mjData* d) {
  int rootid, bodyid, objtype, objid, adr, nusersensor = 0;
  int ne = d->ne, nf = d->nf, nefc = d->nefc;
  mjtNum tmp[6], conforce[6], conray[3];
  mjContact* con;

  // disabled sensors: return
  if (mjDISABLED(mjDSBL_SENSOR)) {
    return;
  }

  // process sensors matching stage
  int rnePost = 0;
  for (int i=0; i < m->nsensor; i++) {
    // skip sensor plugins -- these are handled after builtin sensor types
    if (m->sensor_type[i] == mjSENS_PLUGIN) {
      continue;
    }

    if (m->sensor_needstage[i] == mjSTAGE_ACC) {
      // get sensor info
      mjtSensor type =  m->sensor_type[i];
      objtype = m->sensor_objtype[i];
      objid = m->sensor_objid[i];
      adr = m->sensor_adr[i];

      // call mj_rnePostConstraint when first relevant sensor is encountered
      if (rnePost == 0  && (type == mjSENS_ACCELEROMETER ||
                            type == mjSENS_FORCE         ||
                            type == mjSENS_TORQUE        ||
                            type == mjSENS_FRAMELINACC   ||
                            type == mjSENS_FRAMEANGACC   ||
                            type == mjSENS_USER)) {
        // compute cacc, cfrc_int, cfrc_ext
        mj_rnePostConstraint(m, d);

        // mark computed
        rnePost = 1;
      }

      // process according to type
      switch (type) {
      case mjSENS_TOUCH:                                  // touch
        // extract body data
        bodyid = m->site_bodyid[objid];

        // clear result
        d->sensordata[adr] = 0;

        // find contacts in sensor zone, add normal forces
        for (int j=0; j < d->ncon; j++) {
          // contact pointer, contacting bodies  (-1 for flex)
          con = d->contact + j;
          int conbody[2];
          for (int k=0; k < 2; k++) {
            conbody[k] = (con->geom[k] >= 0) ? m->geom_bodyid[con->geom[k]] : -1;
          }

          // select contacts involving sensorized body
          if (con->efc_address >= 0 && (bodyid == conbody[0] || bodyid == conbody[1])) {
            // get contact force:torque in contact frame
            mj_contactForce(m, d, j, conforce);

            // nothing to do if normal is zero
            if (conforce[0] <= 0) {
              continue;
            }

            // convert contact normal force to global frame, normalize
            mju_scl3(conray, con->frame, conforce[0]);
            mju_normalize3(conray);

            // flip ray direction if sensor is on body2
            if (bodyid == conbody[1]) {
              mju_scl3(conray, conray, -1);
            }

            // add if ray-zone intersection (always true when con->pos inside zone)
            if (mju_rayGeom(d->site_xpos+3*objid, d->site_xmat+9*objid,
                            m->site_size+3*objid, con->pos, conray,
                            m->site_type[objid]) >= 0) {
              d->sensordata[adr] += conforce[0];
            }
          }
        }
        break;

      case mjSENS_ACCELEROMETER:                          // accelerometer
        // tmp = site acceleration, in site frame
        mj_objectAcceleration(m, d, mjOBJ_SITE, objid, tmp, 1);

        // assign linear acceleration
        mju_copy3(d->sensordata+adr, tmp+3);
        break;

      case mjSENS_FORCE:                                  // force
        // extract body data
        bodyid = m->site_bodyid[objid];
        rootid = m->body_rootid[bodyid];

        // tmp = interaction force between body and parent, in site frame
        mju_transformSpatial(tmp, d->cfrc_int+6*bodyid, 1,
                             d->site_xpos+3*objid, d->subtree_com+3*rootid, d->site_xmat+9*objid);

        // assign force
        mju_copy3(d->sensordata+adr, tmp+3);
        break;

      case mjSENS_TORQUE:                                 // torque
        // extract body data
        bodyid = m->site_bodyid[objid];
        rootid = m->body_rootid[bodyid];

        // tmp = interaction force between body and parent, in site frame
        mju_transformSpatial(tmp, d->cfrc_int+6*bodyid, 1,
                             d->site_xpos+3*objid, d->subtree_com+3*rootid, d->site_xmat+9*objid);

        // assign torque
        mju_copy3(d->sensordata+adr, tmp);
        break;

      case mjSENS_ACTUATORFRC:                            // actuatorfrc
        d->sensordata[adr] = d->actuator_force[objid];
        break;

      case mjSENS_JOINTACTFRC:                            // jointactfrc
        d->sensordata[adr] = d->qfrc_actuator[m->jnt_dofadr[objid]];
        break;

      case mjSENS_JOINTLIMITFRC:                          // jointlimitfrc
        d->sensordata[adr] = 0;
        for (int j=ne+nf; j < nefc; j++) {
          if (d->efc_type[j] == mjCNSTR_LIMIT_JOINT && d->efc_id[j] == objid) {
            d->sensordata[adr] = d->efc_force[j];
            break;
          }
        }
        break;

      case mjSENS_TENDONLIMITFRC:                         // tendonlimitfrc
        d->sensordata[adr] = 0;
        for (int j=ne+nf; j < nefc; j++) {
          if (d->efc_type[j] == mjCNSTR_LIMIT_TENDON && d->efc_id[j] == objid) {
            d->sensordata[adr] = d->efc_force[j];
            break;
          }
        }
        break;

      case mjSENS_FRAMELINACC:                            // framelinacc
      case mjSENS_FRAMEANGACC:                            // frameangacc
        // get 6D object acceleration, in global frame
        mj_objectAcceleration(m, d, objtype, objid, tmp, 0);

        // copy linear or angular component
        if (m->sensor_type[i] == mjSENS_FRAMELINACC) {
          mju_copy3(d->sensordata+adr, tmp+3);
        } else {
          mju_copy3(d->sensordata+adr, tmp);
        }
        break;

      case mjSENS_USER:                                   // user
        nusersensor++;
        break;

      default:
        mjERROR("invalid type in ACC stage, sensor %d", i);
      }
    }
  }

  // fill in user sensors if detected
  if (nusersensor && mjcb_sensor) {
    mjcb_sensor(m, d, mjSTAGE_ACC);
  }

  // trigger computation of plugins
  if (m->nplugin) {
    const int nslot = mjp_pluginCount();
    for (int i=0; i < m->nplugin; i++) {
      const int slot = m->plugin[i];
      const mjpPlugin* plugin = mjp_getPluginAtSlotUnsafe(slot, nslot);
      if (!plugin) {
        mjERROR("invalid plugin slot: %d", slot);
      }
      if ((plugin->capabilityflags & mjPLUGIN_SENSOR) && plugin->needstage == mjSTAGE_ACC) {
        if (!plugin->compute) {
          mjERROR("`compute` is null for plugin at slot %d", slot);
        }
        if (rnePost == 0) {
          // compute cacc, cfrc_int, cfrc_ext
          // TODO(b/247107630): add a flag to allow plugin to specify whether it actually needs this
          mj_rnePostConstraint(m, d);

          // mark computed
          rnePost = 1;
        }
        plugin->compute(m, d, i, mjPLUGIN_SENSOR);
      }
    }
  }

  // cutoff
  apply_cutoff(m, d, mjSTAGE_ACC);
}



//-------------------------------- energy ----------------------------------------------------------

// position-dependent energy (potential)
void mj_energyPos(const mjModel* m, mjData* d) {
  int padr;
  mjtNum dif[3], quat[4], stiffness;

  // init potential energy:  -sum_i body(i).mass * mju_dot(body(i).pos, gravity)
  d->energy[0] = 0;
  if (!mjDISABLED(mjDSBL_GRAVITY)) {
    for (int i=1; i < m->nbody; i++) {
      d->energy[0] -= m->body_mass[i] * mju_dot3(m->opt.gravity, d->xipos+3*i);
    }
  }

  // add joint-level springs
  if (!mjDISABLED(mjDSBL_PASSIVE)) {
    for (int i=0; i < m->njnt; i++) {
      stiffness = m->jnt_stiffness[i];
      padr = m->jnt_qposadr[i];

      switch ((mjtJoint) m->jnt_type[i]) {
      case mjJNT_FREE:
        mju_copy4(quat, d->qpos+padr);
        mju_normalize4(quat);
        mju_sub3(dif, quat, m->qpos_spring+padr);
        d->energy[0] += 0.5*stiffness*mju_dot3(dif, dif);

        // continue with rotations
        padr += 3;
        mjFALLTHROUGH;

      case mjJNT_BALL:
        // covert quatertion difference into angular "velocity"
        mju_copy4(quat, d->qpos+padr);
        mju_normalize4(quat);
        mju_subQuat(dif, d->qpos + padr, m->qpos_spring + padr);
        d->energy[0] += 0.5*stiffness*mju_dot3(dif, dif);
        break;

      case mjJNT_SLIDE:
      case mjJNT_HINGE:
        d->energy[0] += 0.5*stiffness*
                        (d->qpos[padr] - m->qpos_spring[padr])*
                        (d->qpos[padr] - m->qpos_spring[padr]);
        break;
      }
    }
  }

  // add tendon-level springs
  if (!mjDISABLED(mjDSBL_PASSIVE)) {
    for (int i=0; i < m->ntendon; i++) {
      stiffness = m->tendon_stiffness[i];
      mjtNum length = d->ten_length[i];
      mjtNum displacement = 0;

      // compute spring displacement
      mjtNum lower = m->tendon_lengthspring[2*i];
      mjtNum upper = m->tendon_lengthspring[2*i+1];
      if (length > upper) {
        displacement = upper - length;
      } else if (length < lower) {
        displacement = lower - length;
      }

      d->energy[0] += 0.5*stiffness*displacement*displacement;
    }
  }

  // add flex-level springs for dim=1 (dim>1 requires plugins)
  if (!mjDISABLED(mjDSBL_PASSIVE)) {
    for (int i=0; i < m->nflex; i++) {
      stiffness = m->flex_edgestiffness[i];
      if (m->flex_rigid[i] || stiffness == 0 || m->flex_dim[i] > 1) {
        continue;
      }

      // process non-rigid edges of this flex
      int flex_edgeadr = m->flex_edgeadr[i];
      int flex_edgenum = m->flex_edgenum[i];
      for (int e=flex_edgeadr; e < flex_edgeadr+flex_edgenum; e++) {
        if (!m->flexedge_rigid[e]) {
          mjtNum displacement = m->flexedge_length0[e] - d->flexedge_length[e];
          d->energy[0] += 0.5*stiffness*displacement*displacement;
        };
      }
    }
  }
}



// velocity-dependent energy (kinetic)
void mj_energyVel(const mjModel* m, mjData* d) {
  mj_markStack(d);
  mjtNum *vec = mjSTACKALLOC(d, m->nv, mjtNum);

  // kinetic energy:  0.5 * qvel' * M * qvel
  mj_mulM(m, d, vec, d->qvel);
  d->energy[1] = 0.5*mju_dot(vec, d->qvel, m->nv);

  mj_freeStack(d);
}
