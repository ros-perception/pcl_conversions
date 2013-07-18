/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Open Source Robotics Foundation, Inc. nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// This header provides stubs for all of the conversion functions which exist
// in the Hydro version of this package. This allows users to use the same
// functions seemlessly between versions of ROS and PCL.

// In the majority of cases in Groovy these functions are simply copies, but
// the move* functions are still destructive.

#ifndef PCL_CONVERSIONS_H__
#define PCL_CONVERSIONS_H__

#include <vector>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/Vertices.h>
#include <pcl/PolygonMesh.h>


namespace pcl_conversions {

  /** PCLHeader <=> Header **/

  inline
  void fromPCL(const std_msgs::Header &pcl_header, std_msgs::Header &header)
  {
    header = pcl_header;
  }

  inline
  void toPCL(const std_msgs::Header &header, std_msgs::Header &pcl_header)
  {
    pcl_header = header;
  }

  inline
  std_msgs::Header fromPCL(const std_msgs::Header &pcl_header)
  {
    return pcl_header;
  }

  inline
  std_msgs::Header toPCL(const std_msgs::Header &header)
  {
    return header;
  }

  /** PCLImage <=> Image **/

  inline
  void copyPCLImageMetaData(const sensor_msgs::Image &pcl_image, sensor_msgs::Image &image)
  {
    fromPCL(pcl_image.header, image.header);
    image.height = pcl_image.height;
    image.width = pcl_image.width;
    image.encoding = pcl_image.encoding;
    image.is_bigendian = pcl_image.is_bigendian;
    image.step = pcl_image.step;
  }

  inline
  void fromPCL(const sensor_msgs::Image &pcl_image, sensor_msgs::Image &image)
  {
    image = pcl_image;
  }

  inline
  void moveFromPCL(sensor_msgs::Image &pcl_image, sensor_msgs::Image &image)
  {
    copyPCLImageMetaData(pcl_image, image);
    image.data.swap(pcl_image.data);
  }

  inline
  void copyImageMetaData(const sensor_msgs::Image &image, sensor_msgs::Image &pcl_image)
  {
    toPCL(image.header, pcl_image.header);
    pcl_image.height = image.height;
    pcl_image.width = image.width;
    pcl_image.encoding = image.encoding;
    pcl_image.is_bigendian = image.is_bigendian;
    pcl_image.step = image.step;
  }

  inline
  void toPCL(const sensor_msgs::Image &image, sensor_msgs::Image &pcl_image)
  {
    pcl_image = image;
  }

  inline
  void moveToPCL(sensor_msgs::Image &image, sensor_msgs::Image &pcl_image)
  {
    copyImageMetaData(image, pcl_image);
    pcl_image.data.swap(image.data);
  }

  /** PCLPointField <=> PointField **/

  inline
  void fromPCL(const sensor_msgs::PointField &pcl_pf, sensor_msgs::PointField &pf)
  {
    pf = pcl_pf;
  }

  inline
  void fromPCL(const std::vector<sensor_msgs::PointField> &pcl_pfs, std::vector<sensor_msgs::PointField> &pfs)
  {
    pfs = pcl_pfs;
  }

  inline
  void toPCL(const sensor_msgs::PointField &pf, sensor_msgs::PointField &pcl_pf)
  {
    pcl_pf = pf;
  }

  inline
  void toPCL(const std::vector<sensor_msgs::PointField> &pfs, std::vector<sensor_msgs::PointField> &pcl_pfs)
  {
    pcl_pfs = pfs;
  }

  /** PCLPointCloud2 <=> PointCloud2 **/

  inline
  void copyPCLPointCloud2MetaData(const sensor_msgs::PointCloud2 &pcl_pc2, sensor_msgs::PointCloud2 &pc2)
  {
    fromPCL(pcl_pc2.header, pc2.header);
    pc2.height = pcl_pc2.height;
    pc2.width = pcl_pc2.width;
    fromPCL(pcl_pc2.fields, pc2.fields);
    pc2.is_bigendian = pcl_pc2.is_bigendian;
    pc2.point_step = pcl_pc2.point_step;
    pc2.row_step = pcl_pc2.row_step;
    pc2.is_dense = pcl_pc2.is_dense;
  }

  inline
  void fromPCL(const sensor_msgs::PointCloud2 &pcl_pc2, sensor_msgs::PointCloud2 &pc2)
  {
    pc2 = pcl_pc2;
  }

  inline
  void moveFromPCL(sensor_msgs::PointCloud2 &pcl_pc2, sensor_msgs::PointCloud2 &pc2)
  {
    copyPCLPointCloud2MetaData(pcl_pc2, pc2);
    pc2.data.swap(pcl_pc2.data);
  }

  inline
  void copyPointCloud2MetaData(const sensor_msgs::PointCloud2 &pc2, sensor_msgs::PointCloud2 &pcl_pc2)
  {
    toPCL(pc2.header, pcl_pc2.header);
    pcl_pc2.height = pc2.height;
    pcl_pc2.width = pc2.width;
    toPCL(pc2.fields, pcl_pc2.fields);
    pcl_pc2.is_bigendian = pc2.is_bigendian;
    pcl_pc2.point_step = pc2.point_step;
    pcl_pc2.row_step = pc2.row_step;
    pcl_pc2.is_dense = pc2.is_dense;
  }

  inline
  void toPCL(const sensor_msgs::PointCloud2 &pc2, sensor_msgs::PointCloud2 &pcl_pc2)
  {
    pcl_pc2 = pc2;
  }

  inline
  void moveToPCL(sensor_msgs::PointCloud2 &pc2, sensor_msgs::PointCloud2 &pcl_pc2)
  {
    copyPointCloud2MetaData(pc2, pcl_pc2);
    pcl_pc2.data.swap(pc2.data);
  }

  /** pcl::PointIndices <=> pcl_msgs::PointIndices **/

  inline
  void fromPCL(const pcl::PointIndices &pcl_pi, pcl::PointIndices &pi)
  {
    pi = pcl_pi;
  }

  inline
  void moveFromPCL(pcl::PointIndices &pcl_pi, pcl::PointIndices &pi)
  {
    fromPCL(pcl_pi.header, pi.header);
    pi.indices.swap(pcl_pi.indices);
  }

  inline
  void toPCL(const pcl::PointIndices &pi, pcl::PointIndices &pcl_pi)
  {
    pcl_pi = pi;
  }

  inline
  void moveToPCL(pcl::PointIndices &pi, pcl::PointIndices &pcl_pi)
  {
    toPCL(pi.header, pcl_pi.header);
    pcl_pi.indices.swap(pi.indices);
  }

  /** pcl::ModelCoefficients <=> pcl_msgs::ModelCoefficients **/

  inline
  void fromPCL(const pcl::ModelCoefficients &pcl_mc, pcl::ModelCoefficients &mc)
  {
    mc = pcl_mc;
  }

  inline
  void moveFromPCL(pcl::ModelCoefficients &pcl_mc, pcl::ModelCoefficients &mc)
  {
    fromPCL(pcl_mc.header, mc.header);
    mc.values.swap(pcl_mc.values);
  }

  inline
  void toPCL(const pcl::ModelCoefficients &mc, pcl::ModelCoefficients &pcl_mc)
  {
    pcl_mc = mc;
  }

  inline
  void moveToPCL(pcl::ModelCoefficients &mc, pcl::ModelCoefficients &pcl_mc)
  {
    toPCL(mc.header, pcl_mc.header);
    pcl_mc.values.swap(mc.values);
  }

  /** pcl::Vertices <=> pcl_msgs::Vertices **/

  inline
  void fromPCL(const pcl::Vertices &pcl_vert, pcl::Vertices &vert)
  {
    vert = pcl_vert;
  }

  inline
  void fromPCL(const std::vector<pcl::Vertices> &pcl_verts, std::vector<pcl::Vertices> &verts)
  {
    verts = pcl_verts;
  }

  inline
  void moveFromPCL(pcl::Vertices &pcl_vert, pcl::Vertices &vert)
  {
    vert.vertices.swap(pcl_vert.vertices);
  }

  inline
  void moveFromPCL(std::vector<pcl::Vertices> &pcl_verts, std::vector<pcl::Vertices> &verts)
  {
    verts.swap(pcl_verts);
  }

  inline
  void toPCL(const pcl::Vertices &vert, pcl::Vertices &pcl_vert)
  {
    pcl_vert.vertices = vert.vertices;
  }

  inline
  void toPCL(const std::vector<pcl::Vertices> &verts, std::vector<pcl::Vertices> &pcl_verts)
  {
    pcl_verts = verts;
  }

  inline
  void moveToPCL(pcl::Vertices &vert, pcl::Vertices &pcl_vert)
  {
    pcl_vert.vertices.swap(vert.vertices);
  }

  inline
  void moveToPCL(std::vector<pcl::Vertices> &verts, std::vector<pcl::Vertices> &pcl_verts)
  {
    pcl_verts.swap(verts);
  }

  /** pcl::PolygonMesh <=> pcl_msgs::PolygonMesh **/

  inline
  void fromPCL(const pcl::PolygonMesh &pcl_mesh, pcl::PolygonMesh &mesh)
  {
    mesh = pcl_mesh;
  }

  inline
  void moveFromPCL(pcl::PolygonMesh &pcl_mesh, pcl::PolygonMesh &mesh)
  {
    fromPCL(pcl_mesh.header, mesh.header);
    moveFromPCL(pcl_mesh.cloud, mesh.cloud);
    moveFromPCL(pcl_mesh.cloud, mesh.cloud);
  }

  inline
  void toPCL(const pcl::PolygonMesh &mesh, pcl::PolygonMesh &pcl_mesh)
  {
    pcl_mesh = mesh;
  }

  inline
  void moveToPCL(pcl::PolygonMesh &mesh, pcl::PolygonMesh &pcl_mesh)
  {
    toPCL(mesh.header, pcl_mesh.header);
    moveToPCL(mesh.cloud, pcl_mesh.cloud);
    moveToPCL(mesh.polygons, pcl_mesh.polygons);
  }

} // namespace pcl_conversions

#endif /* PCL_CONVERSIONS_H__ */