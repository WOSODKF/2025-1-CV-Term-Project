#include "simulator/simulator.hpp"

mjModel* _model;
mjData* _data;
mjvCamera _camera;
mjvOption _option;
mjvScene _scene;
mjrContext _context;
GLFWwindow* _window;

bool _button_left;
bool _button_middle;
bool _button_right;
bool _run;
bool _reset;
double _lastx;
double _lasty;
int _pause_pub_cnt;
int _reset_pub_cnt;

// robot camera
int ROBOT_CAM_WIDTH, ROBOT_CAM_HEIGHT;
mjvCamera _robot_camera;
mjrRect _robot_viewport;

// segmentor init
bool segmentor_inited = false;

// mesh init
bool init_mesh_sent = false;

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(_model, _data);
    mj_forward(_model, _data);
    std::cout << "simulation reset" << std::endl;
    _reset = true;
    _run = false;
  }

  // spacebar: pause or run
  if (act == GLFW_PRESS && key == GLFW_KEY_SPACE) {
    _run = !_run;
    if (_run) {
      std::cout << "Resume simulation" << std::endl;
      _reset = false;
      _pause_pub_cnt = 0;
      _reset_pub_cnt = 0;
    } else
      std::cout << "Simulation paused" << std::endl;
  }
}

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  _button_left =
    (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  _button_middle =
    (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  _button_right =
    (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
  glfwGetCursorPos(window, &_lastx, &_lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  if (!_button_left && !_button_middle && !_button_right)
    return;

  double dx = xpos - _lastx;
  double dy = ypos - _lasty;
  _lastx = xpos;
  _lasty = ypos;

  int width, height;
  glfwGetWindowSize(window, &width, &height);

  bool mod_shift =
    (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  mjtMouse action;
  if (_button_right)
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  else if (_button_left)
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  else
    action = mjMOUSE_ZOOM;

  mjv_moveCamera(_model, action, dx / height, dy / height, &_scene, &_camera);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  mjv_moveCamera(_model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &_scene, &_camera);
}

Simulator::Simulator(std::shared_ptr<config_t> config): _config(config) {
  char* filename = const_cast<char*>(_config->xml_file.c_str());
  char error[1000] = "Could not load binary model";
  _model = mj_loadXML(filename, 0, error, 1000);
  if (!_model)
    mju_error_s("Load model error: %s", error);

  _data = mj_makeData(_model);
  if (!glfwInit())
    mju_error("Could not initialize GLFW");

  _window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
  glfwMakeContextCurrent(_window);
  glfwSwapInterval(1);

  mjv_defaultCamera(&_camera);
  mjv_defaultOption(&_option);
  mjv_defaultScene(&_scene);
  mjr_defaultContext(&_context);
  mjv_makeScene(_model, &_scene, 2000);
  mjr_makeContext(_model, &_context, mjFONTSCALE_150);

  // _model->opt.integrator = mjINT_IMPLICITFAST;  //
  // int left_cam_id = mj_name2id(_model, mjOBJ_CAMERA, "camera_left_0");
  // _camera.type = mjCAMERA_FIXED;
  // _camera.fixedcamid = left_cam_id;

  // visualization option
  // _option.flags[mjVIS_COM] = 1;
  _option.flags[mjVIS_SKIN] = 0;
  _option.flags[mjVIS_CAMERA] = 1;
  // _option.flags[mjVIS_PERTFORCE] = 1;
  // _option.flags[mjVIS_CONTACTFORCE] = 1;
  // _option.flags[mjVIS_CONTACTPOINT] = 1;

  _button_left = false;
  _button_middle = false;
  _button_right = false;
  _run = false;
  _reset = false;
  _lastx = 0;
  _lasty = 0;
  _pause_pub_cnt = 0;
  _reset_pub_cnt = 0;

  glfwSetKeyCallback(_window, keyboard);
  glfwSetCursorPosCallback(_window, mouse_move);
  glfwSetMouseButtonCallback(_window, mouse_button);
  glfwSetScrollCallback(_window, scroll);

  // identify agent numbers
  _agent_num = 0;
  int temp_id = 0;
  std::string body_name;

  while (true) {
    body_name = "base_" + std::to_string(_agent_num);
    temp_id = mj_name2id(_model, mjOBJ_BODY, body_name.c_str());
    if (temp_id == -1)
      break;
    else
      _agent_num++;
  }
  if (_agent_num == 0) {
    throw std::out_of_range("Agent not found");
  }

  // Initialize members
  mj_forward(_model, _data);  // to access body xpos
  ros::NodeHandle node("~");
  _robot = std::vector<std::shared_ptr<Robot>>(_agent_num);

  for (int id = 0; id < _agent_num; id++) {
    // robot instance
    _robot[id] = make_robot(_model, _data, _config, node, id);
  }

  ROBOT_CAM_WIDTH = _config->camera.width;
  ROBOT_CAM_HEIGHT = _config->camera.height;
  _robot_viewport = {0, 0, ROBOT_CAM_WIDTH, ROBOT_CAM_HEIGHT};

  auto first_body_id = mj_name2id(_model, mjOBJ_BODY, "cloth_0");
  if (first_body_id == -1) {
    ROS_ERROR("Cloth body not found");
  }
  _mesh.init(first_body_id, _config->mesh.rows, _config->mesh.cols);
  _mesh_pub = make_mesh_publisher(node, "GT", _mesh);

  _segmentor_init_sub = node.subscribe<std_msgs::Bool>(
    "/segmentor/initialized", 2,
    [&](const std_msgs::Bool::ConstPtr& msg) { segmentor_inited = msg->data; });

  // callback registration
  register_callback();
}

Simulator::~Simulator() {
  mjv_freeScene(&_scene);
  mjr_freeContext(&_context);
  mj_deleteData(_data);
  mj_deleteModel(_model);
}

void Simulator::run() {
  mjtNum last_measure_time = 0.0;
  mjtNum last_view_render_time = 0.0;

  while (!glfwWindowShouldClose(_window) && ros::ok()) {
    auto comp_start = std::chrono::steady_clock::now();

    mjtNum simstart = _data->time;
    while (_data->time - simstart < 1.0 / _config->sim.sim_render_rate &&
           _run) {
      // simulation step
      mj_step(_model, _data);

      // publish states
      for (int id = 0; id < _agent_num; id++) {
        _robot[id]
          ->publish_state();  // (robot state update & control done in callback)
      }
    }

    auto dyna_end = std::chrono::steady_clock::now();

    // get mesh data & publish
    bool init_mesh = !init_mesh_sent &&
      _data->time > _config->mesh.mesh_init_time && segmentor_inited;

    if (init_mesh || (_config->mesh.GT_mesh && segmentor_inited)) {
      _mesh.update_by_mj(_model, _data);
      _mesh_pub->update(_mesh, init_mesh);
      _mesh_pub->pub();
      init_mesh_sent = true;
    }

    // update view & publish
    bool render_view = _config->view.render_view && _run &&
      _data->time > _config->measure.measure_start_time && segmentor_inited;

    if (
      render_view &&
      _data->time - last_view_render_time > 1 / _config->view.view_pub_rate) {
      for (int id = 0; id < _agent_num; id++) {
        _robot[id]->update_view(
          _model, _data, _option, _robot_camera, _scene, _context,
          _robot_viewport, _config->sim.data_gen_mode);
        _robot[id]->publish_view();
      }
      last_view_render_time = _data->time;
    }

    auto robot_render_end = std::chrono::steady_clock::now();

    // update measurement & publish
    bool measure_on = _config->measure.measure_on && _run &&
      _data->time > _config->measure.measure_start_time && segmentor_inited;

    if (
      measure_on &&
      _data->time - last_measure_time > 1 / _config->measure.measure_pub_rate) {
      for (int id = 0; id < _agent_num; id++) {
        _robot[id]->update_wrench(_model, _data);
        _robot[id]->publish_measurement();
      }
      last_measure_time = _data->time;
    }

    // reset behavior
    if (_reset && _reset_pub_cnt == 0) {
      _reset_pub_cnt++;
      _pause_pub_cnt++;

      _reset = false;
    }

    auto robot_measure_end = std::chrono::steady_clock::now();

    // simulation scene rendering
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(_window, &viewport.width, &viewport.height);
    mjv_updateScene(
      _model, _data, &_option, NULL, &_camera, mjCAT_ALL, &_scene);
    mjr_render(viewport, &_scene, &_context);

    // clock visualization
    char time_string[50];
    snprintf(time_string, sizeof(time_string), "%.3f s", _data->time);
    mjr_overlay(
      mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, "Sim Time", time_string,
      &_context);

    glfwSwapBuffers(_window);
    glfwPollEvents();

    auto scene_render_end = std::chrono::steady_clock::now();

    // duration check
    auto dyna_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      dyna_end - comp_start);
    auto robot_render_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        robot_render_end - dyna_end);
    auto measure_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        robot_measure_end - robot_render_end);
    auto scene_render_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        scene_render_end - robot_render_end);
    auto net_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      scene_render_end - comp_start);
    if (_run && _config->sim.duration_check) {
      std::cout << "[Durations]------" << std::endl;
      std::cout << "sim time: " << _data->time << "s" << std::endl
                << "dynamics: " << dyna_duration.count() << "ms" << std::endl
                << "robot rendering: " << robot_render_duration.count() << "ms"
                << std::endl
                << "measurement acquisition:" << measure_duration.count()
                << "ms" << std::endl
                << "scene rendering: " << scene_render_duration.count() << "ms"
                << std::endl
                << "net: " << net_duration.count() << "ms" << std::endl
                << std::endl;
    }
  }

  if (glfwWindowShouldClose(_window)) {
    ROS_INFO("GLFW window closed. Shutting down ROS.");
    ros::shutdown();
  }
}

void Simulator::callback(const mjModel* m, mjData* d) {
  for (int id = 0; id < _agent_num; id++) {
    _robot[id]->set_mujoco_control(m, d);
  }
}

void Simulator::register_callback() {
  s_callback = std::bind(
    &Simulator::callback, this, std::placeholders::_1, std::placeholders::_2);
  mjcb_control = &Simulator::callback_wrapper;
}

void Simulator::callback_wrapper(const mjModel* m, mjData* d) {
  if (s_callback) {
    s_callback(m, d);
  }
}