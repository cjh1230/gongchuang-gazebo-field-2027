#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

namespace gongchuang {
class TurntableSystem : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate {
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity joint{gz::sim::kNullEntity};
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  double speed{}, dwell{}, initial{}, deadline{}, target{}, command{}, lastTime{-1}, lastPub{-1};
  double step{};
  int direction{}, stopCount{};
  bool initialized{false}, waiting{true};
 public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    model=gz::sim::Model(entity);
    const double period=sdf->Get<double>("seconds_per_revolution");
    dwell=sdf->Get<double>("dwell_seconds"); initial=sdf->Get<double>("initial_dwell_seconds");
    stopCount=sdf->Get<int>("stops_per_revolution");
    direction=sdf->Get<int>("direction");
    if (period<6 || period>10 || dwell<0 || initial<0 || stopCount<1 || std::abs(direction)>1)
      throw std::runtime_error("Invalid turntable parameters");
    if (!direction) { std::mt19937 rng(sdf->Get<unsigned>("seed")); direction=(rng()%2)?1:-1; }
    speed=2*std::acos(-1.)/period; step=2*std::acos(-1.)/stopCount;
    publisher=node.Advertise<gz::msgs::StringMsg>("/gongchuang/turntable/status");
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (joint==gz::sim::kNullEntity) {
      joint=model.JointByName(ecm,"turntable_joint");
      if (joint==gz::sim::kNullEntity) return;
      ecm.CreateComponent(joint,gz::sim::components::JointPosition());
      ecm.CreateComponent(joint,gz::sim::components::JointVelocity());
    }
    if (info.paused) return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    const double dt=std::chrono::duration<double>(info.dt).count();
    const auto *pos=ecm.Component<gz::sim::components::JointPosition>(joint);
    if (!pos || pos->Data().empty() || dt<=0) return;
    const double angle=pos->Data()[0];
    if (!initialized || now<lastTime) {
      initialized=true; waiting=true; target=angle; deadline=now+initial;
    }
    lastTime=now;
    if (waiting && now>=deadline) { waiting=false; target+=direction*step; }
    double remaining=direction*(target-angle);
    if (!waiting && remaining<=1e-5) { waiting=true; deadline=now+dwell; }
    command=waiting?0.:direction*std::min(speed,std::max(0.,remaining)/dt);
    ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint,{command});
  }
  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override {
    const double now=std::chrono::duration<double>(info.simTime).count();
    if (info.paused || !initialized || (now-lastPub<.05 && now>=lastPub)) return;
    lastPub=now;
    const auto *p=ecm.Component<gz::sim::components::JointPosition>(joint);
    const auto *v=ecm.Component<gz::sim::components::JointVelocity>(joint);
    if (!p || p->Data().empty()) return;
    std::ostringstream out;
    out<<std::setprecision(12)<<"{\"sim_time\":"<<now<<",\"angle\":"<<p->Data()[0]
       <<",\"velocity\":"<<((v && !v->Data().empty())?v->Data()[0]:0.)
       <<",\"command\":"<<command<<",\"direction\":"<<direction
       <<",\"state\":\""<<(waiting?"dwell":"rotating")<<"\",\"target\":"<<target<<"}";
    gz::msgs::StringMsg msg; msg.set_data(out.str()); publisher.Publish(msg);
  }
};
}
GZ_ADD_PLUGIN(gongchuang::TurntableSystem, gz::sim::System,
    gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
