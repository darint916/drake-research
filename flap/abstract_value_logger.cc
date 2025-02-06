#include <drake/systems/framework/leaf_system.h>
#include <fstream>
#include <vector>
#include <string>

class AbstractValueLogger : public drake::systems::LeafSystem<double> {
 public:
  AbstractValueLogger(const drake::AbstractValue& model_value, double publish_period_seconds) {
    this->DeclareAbstractInputPort("abstract_value", model_value.Clone());
    this->DeclarePeriodicEvent(publish_period_seconds, 0, 
        [this](const drake::systems::Context<double>& context, 
               const drake::systems::PublishEvent<double>& event) {
          return this->Publish(context);
        });
  }

 private:
  EventStatus Publish(const drake::systems::Context<double>& context) {
    sample_times_.push_back(context.get_time());
    values_.push_back(this->EvalAbstractInput(context, 0)->Clone());
    return EventStatus::Succeeded();
  }

  void WriteCSV(const std::string& file_name) {
    std::ofstream file(file_name);
    for (size_t i = 0; i < sample_times_.size(); ++i) {
      file << sample_times_[i] << "," << values_[i]->to_string() << std::endl;
    }
  }

  std::vector<double> sample_times_;
  std::vector<std::unique_ptr<drake::AbstractValue>> values_;
};