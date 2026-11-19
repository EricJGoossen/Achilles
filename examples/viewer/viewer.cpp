// The viewer: any scene, watched. It loads a .arow (and, optionally, its
// sim_config.yaml), runs it in real time like every other example
// (examples/shared/real_time.hpp), and opens one window for every sensor
// that has a display -- a camera its image, a lidar its scan, and any other
// sensor its readings over time -- each titled by its name. Actuators get
// none: a scene's own program drives them. For a scene without a program of
// its own, or one being built.
//
// Usage: viewer <scene.arow> [sim_config.yaml] [--dt seconds]
//               [--integrator NAME] [--seconds N]
// Semi-implicit Euler at 2 ms by default: the only integrator with contact,
// so a body resting on the ground stays on it. A scene without contact
// (the arms) can ask for --integrator rk4 to keep its energy far better.
// --seconds stops after N simulated seconds; otherwise it runs until every
// window is closed. Every camera's window flies it (display::FlyCamera):
// W/A/S/D to move, E/Q up and down, Shift for faster, a drag to look around
// -- by setting the pose of the joint it's attached to outright, so a camera
// to fly wants a welded joint of its own (each example's camera_mount).
#include <concepts>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <optional>
#include <ostream>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

#include "../shared/flying_camera.hpp"
#include "../shared/real_time.hpp"
#include "algorithms/integrators.hpp"
#include "display/renderer.hpp"
#include "display/views/camera/camera_view.hpp"
#include "display/views/lidar_view.hpp"
#include "display/views/reading_view.hpp"
#include "display/window_set.hpp"
#include "domain/attachment/attachment_contract.hpp"
#include "domain/sensing/camera.hpp"
#include "domain/sensing/lidar.hpp"
#include "interface/simulation.hpp"

namespace {

using achilles::interface::Simulation;

struct Args {
  std::string arow_path;
  std::string config_path;
  float dt = 0.002F;
  Simulation::Integrator integrator =
      Simulation::Integrator::kSemiImplicitEuler;
  float seconds = 0.0F;  // 0: run until every window is closed
};

// Every --integrator name, joined by `separator`.
void PrintIntegratorNames(std::ostream& out, std::string_view separator) {
  std::string_view between;
  for (const achilles::algorithms::IntegratorEntry& entry :
       achilles::algorithms::kIntegrators) {
    out << between << entry.name;
    between = separator;
  }
}

// The arguments, or nothing, with what was wrong printed.
std::optional<Args> ParseArgs(int argc, char** argv) {
  const std::span<char*> argv_span(argv, static_cast<std::size_t>(argc));
  Args args;
  for (std::size_t i = 1; i < argv_span.size(); ++i) {
    const std::string_view arg = argv_span[i];
    const bool has_value = i + 1 < argv_span.size();
    if (arg == "--dt" && has_value) {
      args.dt = std::strtof(argv_span[++i], nullptr);
    } else if (arg == "--seconds" && has_value) {
      args.seconds = std::strtof(argv_span[++i], nullptr);
    } else if (arg == "--integrator" && has_value) {
      const std::optional<Simulation::Integrator> integrator =
          achilles::algorithms::IntegratorFromName(argv_span[++i]);
      if (!integrator.has_value()) {
        std::cerr << "viewer: --integrator must be one of ";
        PrintIntegratorNames(std::cerr, ", ");
        std::cerr << '\n';
        return std::nullopt;
      }
      args.integrator = *integrator;
    } else if (arg.starts_with("--")) {
      std::cerr << "viewer: unknown or incomplete option " << arg << '\n';
      return std::nullopt;
    } else if (args.arow_path.empty()) {
      args.arow_path = arg;
    } else if (args.config_path.empty()) {
      args.config_path = arg;
    } else {
      std::cerr << "viewer: one scene and one config at most\n";
      return std::nullopt;
    }
  }
  if (args.arow_path.empty()) {
    return std::nullopt;
  }
  return args;
}

// "name", or "name #i" past an archetype's first instance: a window's title.
std::string Label(std::string_view name, std::size_t instance) {
  std::string label(name);
  if (instance > 0) {
    label += " #" + std::to_string(instance);
  }
  return label;
}

// Puts up, for every sensor of `sim`, the display that shows it: a camera
// its image, a lidar its scan, and any other sensor whose reading has
// channels its readings over time -- each camera flown by its window.
// Returns how many it put up.
std::size_t AttachDisplays(
    achilles::display::Renderer& renderer, const Simulation& sim
) {
  namespace display = achilles::display;
  namespace sensing = achilles::domain::sensing;
  std::size_t attached = 0;
  const auto attach = [&](auto shown, const auto& attachment) {
    ++attached;
    return renderer.Attach(
        std::move(shown), Label(attachment.name, attachment.instance)
    );
  };
  std::apply(
      [&](const auto&... lists) {
        (
            [&] {
              for (const auto& attachment : lists) {
                using K =
                    typename std::remove_cvref_t<decltype(attachment.handle
                    )>::Kind;
                if constexpr (std::same_as<K, sensing::Camera>) {
                  const display::WindowId window = attach(
                      display::CameraView(attachment.handle), attachment
                  );
                  renderer.Control(
                      window, achilles::examples::FlyCameraFor(sim, attachment)
                  );
                } else if constexpr (std::same_as<K, sensing::Lidar>) {
                  attach(display::LidarView(attachment.handle), attachment);
                } else if constexpr (achilles::domain::attachment::SensorLike<
                                         K> &&
                                     display::HasChannels<K>) {
                  attach(
                      display::ReadingView<K>(attachment.handle), attachment
                  );
                }
              }
            }(),
            ...
        );
      },
      sim.Attachments()
  );
  return attached;
}

int Run(const Args& args) {
  Simulation sim;
  if (!sim.LoadArowFile(args.arow_path) ||
      (!args.config_path.empty() && !sim.LoadConfigFile(args.config_path)) ||
      !sim.Init()) {
    return 1;
  }
  sim.SetIntegrator(args.integrator);

  float sim_time = 0.0F;
  achilles::examples::RunInRealTime(
      "Achilles: " + args.arow_path,
      args.dt,
      [&](achilles::display::Renderer& renderer) {
        if (AttachDisplays(renderer, sim) == 0) {
          std::cerr << "viewer: nothing in this scene has a display (add a "
                       "camera to see it)\n";
        }
      },
      [&] {
        sim_time += args.dt;
        return sim.Step(args.dt);
      },
      [&] { return args.seconds > 0.0F && sim_time >= args.seconds; }
  );
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  const std::optional<Args> args = ParseArgs(argc, argv);
  if (!args.has_value()) {
    std::cerr << "usage: viewer <scene.arow> [sim_config.yaml] [--dt seconds] "
                 "[--integrator ";
    PrintIntegratorNames(std::cerr, "|");
    std::cerr << "] [--seconds N]\n";
    return 1;
  }
  try {
    return Run(*args);
  } catch (const std::exception& e) {
    std::cerr << "viewer: " << e.what() << '\n';
    return 1;
  }
}
