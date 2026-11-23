// Persistent, file-based request protocol; no ROS and no pose publication.
#include <cpu_bbs3d/bbs3d.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

std::vector<Eigen::Vector3d> load(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open point file");
  std::vector<Eigen::Vector3d> points; float p[3];
  while(f.read(reinterpret_cast<char*>(p),12)) points.emplace_back(p[0],p[1],p[2]);
  if (points.empty() || f.gcount()!=0) throw std::runtime_error("invalid point file");
  return points;
}
int main(int argc,char**argv) {
  if(argc!=3) return 2;
  try {
    cpu::BBS3D b; std::ifstream config(argv[2]);
    Eigen::Vector3d lo,hi;
    double resolution,minimum; int levels,threads,timeout;
    for(int i=0;i<3;i++) config>>lo[i];
    for(int i=0;i<3;i++) config>>hi[i];
    config>>resolution>>levels>>threads>>timeout>>minimum>>b.competition_gap
          >>b.peak_distance>>b.peak_angle>>b.maximum_peaks;
    if(!config) return 3;
    b.set_num_threads(threads); b.set_tar_points(load(argv[1]),resolution,levels);
    b.set_trans_search_range(lo,hi);
    b.set_angular_search_range(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,2*M_PI));
    b.set_score_threshold_percentage(minimum); b.enable_timeout();
    b.set_timeout_duration_in_msec(timeout);
    std::cout<<"READY"<<std::endl;
    std::string path;
    while(std::getline(std::cin,path)) {
      b.set_src_points(load(path)); b.localize();
      std::cout<<std::setprecision(14)<<"{\"ms\":"<<b.get_elapsed_time()
               <<",\"complete\":"<<(b.competition_complete?"true":"false")
               <<",\"timed_out\":"<<(b.has_timed_out()?"true":"false")
               <<",\"peak_limit\":"<<(b.peak_limit_reached?"true":"false")
               <<",\"peaks\":[";
      for(size_t k=0;k<b.peaks.size();k++) {
        if(k)std::cout<<',';
        const auto &peak=b.peaks[k];
        std::cout<<"{\"score\":"<<double(peak.score)/b.get_src_points().size()<<",\"pose\":[";
        for(int i=0;i<4;i++){if(i)std::cout<<',';std::cout<<'[';
          for(int j=0;j<4;j++){if(j)std::cout<<',';std::cout<<peak.pose(i,j);} std::cout<<']';}
        std::cout<<"]}";
      }
      std::cout<<"]}"<<std::endl;
    }
  }catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 4;}
}
