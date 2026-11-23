// Offline first-occupied-voxel ray casting. No ROS, no camera, no interpolation of missing surfaces.
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <unordered_set>
#include <vector>
struct Cell {int x,y,z;bool operator==(const Cell&b)const{return x==b.x&&y==b.y&&z==b.z;}};
struct Hash {size_t operator()(const Cell&a)const{return (uint64_t(a.x)*73856093)^(uint64_t(a.y)*19349663)^(uint64_t(a.z)*83492791);}};
int main(int argc,char**argv){
 if(argc!=10){std::cerr<<"map_float32.bin poses.txt out.bin voxel range blind azstep elstep offset_base.txt\n";return 2;}
 const double v=std::stod(argv[4]),range=std::stod(argv[5]),blind=std::stod(argv[6]),azstep=std::stod(argv[7]),elstep=std::stod(argv[8]);
 std::array<double,3> offset;{std::ifstream f(argv[9]);for(auto&x:offset)f>>x;if(!f)return 3;}
 std::unordered_set<Cell,Hash> occ;{std::ifstream f(argv[1],std::ios::binary);std::array<float,3> p;while(f.read((char*)p.data(),12))occ.insert({int(floor(p[0]/v)),int(floor(p[1]/v)),int(floor(p[2]/v))});}
 std::ifstream poses(argv[2]);std::ofstream out(argv[3],std::ios::binary);std::array<double,3> o;std::array<double,9> R;int count=0;
 while(poses>>o[0]>>o[1]>>o[2]){
  for(auto&x:R)poses>>x;if(!poses)return 4;std::vector<float> hits;
  for(double el=-7;el<=52+1e-6;el+=elstep)for(double az=0;az<360-1e-6;az+=azstep){
   double a=az*M_PI/180,e=el*M_PI/180;std::array<double,3>b={cos(e)*cos(a),cos(e)*sin(a),sin(e)},d;
   for(int j=0;j<3;j++)d[j]=R[j*3]*b[0]+R[j*3+1]*b[1]+R[j*3+2]*b[2];
   std::array<int,3> c,step;std::array<double,3> next,delta;
   for(int j=0;j<3;j++){c[j]=int(floor(o[j]/v));step[j]=d[j]>=0?1:-1;delta[j]=fabs(d[j])>1e-12?v/fabs(d[j]):1e30;next[j]=fabs(d[j])>1e-12?((c[j]+(step[j]>0?1:0))*v-o[j])/d[j]:1e30;}
   double t=0;
   while(t<=range){
    if(occ.count({c[0],c[1],c[2]})){
     if(t>=blind){std::array<double,3>w;for(int j=0;j<3;j++)w[j]=d[j]*(t+1e-4);for(int j=0;j<3;j++)hits.push_back(float(R[j]*w[0]+R[3+j]*w[1]+R[6+j]*w[2]+offset[j]));}
     break; // Near-field occluders block farther returns, even inside the blind zone.
    }
    int k=next[0]<next[1]?(next[0]<next[2]?0:2):(next[1]<next[2]?1:2);t=next[k];next[k]+=delta[k];c[k]+=step[k];
   }
  }
  uint32_t n=hits.size()/3;out.write((char*)&n,4);out.write((char*)hits.data(),hits.size()*4);count++;
 }
 std::cerr<<"Rendered "<<count<<" poses using "<<occ.size()<<" occupied voxels\n";
 return 0;
}
