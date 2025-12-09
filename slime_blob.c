#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#define NUM_POINTS 24
#define RADIUS 12.0f
#define POINT_MASS 0.5f
#define VISCOSITY 0.3f
#define FRICTION 0.92f
#define K_SPRING 0.02f
#define K_PRESSURE 0.15f
#define K_BOUNDARY 0.7f
#define METABALL_THRESHOLD 1.8f
#define MAX_VELOCITY 6.0f
#define CONSTRAINT_ITERATIONS 2
#define WOBBLE_STRENGTH 0.3f
#define IRREGULARITY 3.0f
#define GRAVITY_STRENGTH 0.08f
#define WIND_STRENGTH 0.15f
typedef struct {
  float x, y;
} Vec2;
typedef struct {
  Vec2 pos;
  Vec2 old_pos;
  Vec2 acc;
  Vec2 vel;
  float mass;
  float rest_angle;
  float noise_phase;
  int pinned;
} Point;
typedef struct {
  float gravity;
  float wind_x;
  float wind_y;
  float turbulence;
  int enabled;
} Environment;
typedef enum { MODE_GENTLE, MODE_NORMAL, MODE_STRONG, MODE_EXTREME } ForceMode;
Point points[NUM_POINTS];
Point center;
int width, height;
struct termios orig_termios;
Environment env = {0.0f, 0.0f, 0.0f, 0.0f, 0};
ForceMode force_mode = MODE_NORMAL;
int use_color = 1;
int show_glow = 1;
int show_highlights = 1;
float pulse_phase = 0.0f;
float global_wobble = 0.0f;
float slime_viscosity_visual = 1.0f;
int color_theme = 0;
void disableRawMode() {
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
  printf("\033[?25h\033[0m");
}
void enableRawMode() {
  tcgetattr(STDIN_FILENO, &orig_termios);
  atexit(disableRawMode);
  struct termios raw = orig_termios;
  raw.c_lflag &= ~(ECHO | ICANON);
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
  printf("\033[?25l");
}
void get_term_size() {
  struct winsize w;
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
  width = w.ws_col;
  height = w.ws_row;
}
int kbhit() {
  struct timeval tv = {0L, 0L};
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(0, &fds);
  return select(1, &fds, NULL, NULL, &tv) > 0;
}
Vec2 vec_new(float x, float y) { return (Vec2){x, y}; }
Vec2 vec_sub(Vec2 a, Vec2 b) { return (Vec2){a.x - b.x, a.y - b.y}; }
Vec2 vec_add(Vec2 a, Vec2 b) { return (Vec2){a.x + b.x, a.y + b.y}; }
Vec2 vec_scale(Vec2 v, float s) { return (Vec2){v.x * s, v.y * s}; }
float vec_len(Vec2 v) { return sqrtf(v.x * v.x + v.y * v.y); }
float vec_len_sq(Vec2 v) { return v.x * v.x + v.y * v.y; }
Vec2 vec_normalize(Vec2 v) {
  float len = vec_len(v);
  return len > 0.01f ? vec_scale(v, 1.0f / len) : vec_new(0, 0);
}
Vec2 vec_rotate(Vec2 v, float angle) {
  float c = cosf(angle), s = sinf(angle);
  return vec_new(v.x * c - v.y * s, v.x * s + v.y * c);
}
float clamp(float v, float min, float max) {
  return v < min ? min : (v > max ? max : v);
}
float lerp(float a, float b, float t) { return a + (b - a) * t; }
float randf() { return ((float)rand() / RAND_MAX) * 2.0f - 1.0f; }
void init_blob() {
  center.pos = vec_new(width / 2.0f, height / 2.0f);
  center.old_pos = center.pos;
  center.vel = vec_new(0, 0);
  center.acc = vec_new(0, 0);
  center.mass = POINT_MASS;
  center.pinned = 0;
  for (int i = 0; i < NUM_POINTS; i++) {
    float angle = (float)i / NUM_POINTS * 2.0f * M_PI;
    float r = RADIUS + randf() * IRREGULARITY;
    angle += randf() * 0.3f;
    points[i].pos.x = center.pos.x + cosf(angle) * r * 2.0f;
    points[i].pos.y = center.pos.y + sinf(angle) * r;
    points[i].old_pos = points[i].pos;
    points[i].vel = vec_new(0, 0);
    points[i].acc = vec_new(0, 0);
    points[i].mass = POINT_MASS + randf() * 0.2f;
    points[i].rest_angle = angle;
    points[i].noise_phase = randf() * M_PI * 2.0f;
    points[i].pinned = 0;
  }
}
void apply_force_to_point(Point *p, Vec2 force) {
  if (!p->pinned) {
    p->acc = vec_add(p->acc, vec_scale(force, 1.0f / p->mass));
  }
}
void apply_force_global(Vec2 force) {
  float mult = (force_mode == MODE_GENTLE)    ? 0.2f
               : (force_mode == MODE_STRONG)  ? 2.5f
               : (force_mode == MODE_EXTREME) ? 5.0f
                                              : 1.0f;
  force = vec_scale(force, mult);
  for (int i = 0; i < NUM_POINTS; i++) {
    Vec2 varied_force = vec_add(force, vec_new(randf() * 0.5f, randf() * 0.5f));
    apply_force_to_point(&points[i], varied_force);
  }
  apply_force_to_point(&center, force);
}
void apply_radial_force(float strength) {
  for (int i = 0; i < NUM_POINTS; i++) {
    Vec2 dir = vec_sub(points[i].pos, center.pos);
    float dist = vec_len(dir);
    if (dist > 0.1f) {
      Vec2 force = vec_scale(dir, strength / dist);
      apply_force_to_point(&points[i], force);
    }
  }
}
void apply_rotation_force(float angular_vel) {
  for (int i = 0; i < NUM_POINTS; i++) {
    Vec2 r = vec_sub(points[i].pos, center.pos);
    Vec2 tangent = vec_new(-r.y, r.x);
    Vec2 force = vec_scale(vec_normalize(tangent), angular_vel);
    apply_force_to_point(&points[i], force);
  }
}
void apply_squeeze(int axis) {
  for (int i = 0; i < NUM_POINTS; i++) {
    Vec2 offset = vec_sub(points[i].pos, center.pos);
    Vec2 force;
    if (axis == 0) {
      force = vec_new(-offset.x * 0.3f, offset.y * 0.1f);
    } else {
      force = vec_new(offset.x * 0.1f, -offset.y * 0.3f);
    }
    apply_force_to_point(&points[i], force);
  }
}
void apply_stretch(int axis) {
  for (int i = 0; i < NUM_POINTS; i++) {
    Vec2 offset = vec_sub(points[i].pos, center.pos);
    Vec2 force;
    if (axis == 0) {
      force = vec_new(offset.x * 0.3f, -offset.y * 0.1f);
    } else {
      force = vec_new(-offset.x * 0.1f, offset.y * 0.3f);
    }
    apply_force_to_point(&points[i], force);
  }
}
void apply_vibration(float intensity) {
  for (int i = 0; i < NUM_POINTS; i++) {
    Vec2 random_force = vec_new(randf() * intensity, randf() * intensity);
    apply_force_to_point(&points[i], random_force);
  }
}
void poke_random() {
  int idx = rand() % NUM_POINTS;
  Vec2 dir = vec_sub(center.pos, points[idx].pos);
  Vec2 force = vec_scale(vec_normalize(dir), 8.0f);
  apply_force_to_point(&points[idx], force);
}
void apply_directional_stretch(float angle, float strength) {
  Vec2 dir = vec_new(cosf(angle), sinf(angle));
  for (int i = 0; i < NUM_POINTS; i++) {
    Vec2 offset = vec_sub(points[i].pos, center.pos);
    float alignment = offset.x * dir.x + offset.y * dir.y;
    Vec2 force = vec_scale(dir, alignment * strength * 0.1f);
    apply_force_to_point(&points[i], force);
  }
}
void apply_wave(float phase) {
  for (int i = 0; i < NUM_POINTS; i++) {
    float wave = sinf((float)i / NUM_POINTS * M_PI * 4.0f + phase);
    Vec2 force = vec_new(wave * 0.5f, 0);
    apply_force_to_point(&points[i], force);
  }
}
void apply_slime_wobble(float dt) {
  for (int i = 0; i < NUM_POINTS; i++) {
    points[i].noise_phase += dt * 2.0f;
    float wobble_x = sinf(points[i].noise_phase) * WOBBLE_STRENGTH;
    float wobble_y = cosf(points[i].noise_phase * 1.3f) * WOBBLE_STRENGTH;
    Vec2 wobble_force = vec_new(wobble_x, wobble_y);
    apply_force_to_point(&points[i], wobble_force);
  }
}
void apply_environmental_forces() {
  if (!env.enabled)
    return;
  Vec2 env_force = vec_new(env.wind_x, env.gravity + env.wind_y);
  for (int i = 0; i < NUM_POINTS; i++) {
    Vec2 force = env_force;
    if (env.turbulence > 0) {
      force = vec_add(
          force, vec_new(randf() * env.turbulence, randf() * env.turbulence));
    }
    apply_force_to_point(&points[i], force);
  }
  apply_force_to_point(&center, env_force);
}
void solve_constraints() {
  for (int iter = 0; iter < CONSTRAINT_ITERATIONS; iter++) {
    for (int i = 0; i < NUM_POINTS; i++) {
      int prev = (i - 1 + NUM_POINTS) % NUM_POINTS;
      int next = (i + 1) % NUM_POINTS;
      float rest_len = (RADIUS * 2.5f * M_PI) / NUM_POINTS;
      Vec2 delta = vec_sub(points[prev].pos, points[i].pos);
      float dist = vec_len(delta);
      if (dist > 0.01f) {
        float diff = (dist - rest_len) / dist;
        Vec2 correction = vec_scale(delta, diff * 0.2f);
        if (!points[i].pinned)
          points[i].pos = vec_add(points[i].pos, correction);
        if (!points[prev].pinned)
          points[prev].pos = vec_sub(points[prev].pos, correction);
      }
      Vec2 to_center = vec_sub(center.pos, points[i].pos);
      dist = vec_len(to_center);
      float target_dist = RADIUS * 1.2f;
      if (dist > target_dist * 1.5f && dist > 0.01f) {
        float diff = (dist - target_dist) / dist;
        Vec2 correction = vec_scale(to_center, diff * 0.15f);
        if (!points[i].pinned)
          points[i].pos = vec_add(points[i].pos, correction);
      }
    }
  }
}
void apply_boundary_constraints(Point *p) {
  float margin = 2.0f;
  Vec2 vel = p->vel;
  if (p->pos.x < margin) {
    p->pos.x = margin;
    p->vel.x *= -K_BOUNDARY;
  }
  if (p->pos.x >= width - margin) {
    p->pos.x = width - margin;
    p->vel.x *= -K_BOUNDARY;
  }
  if (p->pos.y < margin) {
    p->pos.y = margin;
    p->vel.y *= -K_BOUNDARY;
  }
  if (p->pos.y >= height - margin) {
    p->pos.y = height - margin;
    p->vel.y *= -K_BOUNDARY;
  }
}
void update_physics(float dt) {
  apply_slime_wobble(dt);
  for (int i = 0; i < NUM_POINTS; i++) {
    Point *p = &points[i];
    int prev = (i - 1 + NUM_POINTS) % NUM_POINTS;
    int next = (i + 1) % NUM_POINTS;
    Vec2 d_prev = vec_sub(points[prev].pos, p->pos);
    float dist = vec_len(d_prev);
    float rest = (RADIUS * 2.5f * M_PI) / NUM_POINTS;
    if (dist > 0.01f) {
      Vec2 force = vec_scale(d_prev, K_SPRING * (dist - rest) / dist);
      apply_force_to_point(p, force);
    }
    Vec2 d_next = vec_sub(points[next].pos, p->pos);
    dist = vec_len(d_next);
    if (dist > 0.01f) {
      Vec2 force = vec_scale(d_next, K_SPRING * (dist - rest) / dist);
      apply_force_to_point(p, force);
    }
    Vec2 to_center = vec_sub(center.pos, p->pos);
    dist = vec_len(to_center);
    if (dist > 0.01f) {
      Vec2 force =
          vec_scale(to_center, K_SPRING * 0.15f * (dist - RADIUS) / dist);
      apply_force_to_point(p, force);
    }
    if (dist < RADIUS * 0.5f && dist > 0.01f) {
      Vec2 repulsion = vec_scale(to_center, -K_PRESSURE / dist);
      apply_force_to_point(p, repulsion);
    }
    p->acc = vec_add(p->acc, vec_scale(p->vel, -VISCOSITY));
  }
  Vec2 avg_pos = vec_new(0, 0);
  for (int i = 0; i < NUM_POINTS; i++) {
    avg_pos = vec_add(avg_pos, points[i].pos);
  }
  avg_pos = vec_scale(avg_pos, 1.0f / NUM_POINTS);
  Vec2 center_force = vec_scale(vec_sub(avg_pos, center.pos), 0.1f);
  apply_force_to_point(&center, center_force);
  apply_environmental_forces();
  for (int i = 0; i < NUM_POINTS; i++) {
    Point *p = &points[i];
    if (p->pinned)
      continue;
    p->vel = vec_add(p->vel, vec_scale(p->acc, dt));
    float vel_mag = vec_len(p->vel);
    if (vel_mag > MAX_VELOCITY) {
      p->vel = vec_scale(p->vel, MAX_VELOCITY / vel_mag);
    }
    p->vel = vec_scale(p->vel, FRICTION);
    p->old_pos = p->pos;
    p->pos = vec_add(p->pos, vec_scale(p->vel, dt));
    p->acc = vec_new(0, 0);
    apply_boundary_constraints(p);
  }
  if (!center.pinned) {
    center.vel = vec_add(center.vel, vec_scale(center.acc, dt));
    float vel_mag = vec_len(center.vel);
    if (vel_mag > MAX_VELOCITY) {
      center.vel = vec_scale(center.vel, MAX_VELOCITY / vel_mag);
    }
    center.vel = vec_scale(center.vel, FRICTION);
    center.old_pos = center.pos;
    center.pos = vec_add(center.pos, vec_scale(center.vel, dt));
    center.acc = vec_new(0, 0);
    apply_boundary_constraints(&center);
  }
  solve_constraints();
  global_wobble += dt;
  pulse_phase += dt * 3.0f;
}
const char *get_slime_color(float density, float highlight, float glow) {
  if (!use_color)
    return "";
  if (highlight > 0.8f && show_highlights) {
    return "\033[97;1m";
  }
  if (show_glow && density < 3.5f && density > METABALL_THRESHOLD) {
    if (color_theme == 0)
      return "\033[92m";
    if (color_theme == 1)
      return "\033[95m";
    if (color_theme == 2)
      return "\033[96m";
  }
  switch (color_theme) {
  case 0:
    if (density > 8.0f)
      return "\033[32;1m";
    if (density > 6.0f)
      return "\033[32m";
    if (density > 4.0f)
      return "\033[92m";
    if (density > 2.5f)
      return "\033[36m";
    return "\033[32m";
  case 1:
    if (density > 8.0f)
      return "\033[35;1m";
    if (density > 6.0f)
      return "\033[35m";
    if (density > 4.0f)
      return "\033[95m";
    if (density > 2.5f)
      return "\033[94m";
    return "\033[35m";
  case 2:
    if (density > 8.0f)
      return "\033[36;1m";
    if (density > 6.0f)
      return "\033[36m";
    if (density > 4.0f)
      return "\033[96m";
    if (density > 2.5f)
      return "\033[34m";
    return "\033[36m";
  case 3: {
    int hue = (int)(pulse_phase * 50.0f + density * 20.0f) % 6;
    const char *colors[] = {"\033[31m", "\033[33m", "\033[32m",
                            "\033[36m", "\033[34m", "\033[35m"};
    return colors[hue];
  }
  }
  return "\033[32m";
}
void render() {
  static char buffer[200][300];
  static char color_buffer[200][300][20];
  for (int y = 0; y < height && y < 200; y++) {
    for (int x = 0; x < width && x < 300; x++) {
      buffer[y][x] = ' ';
      color_buffer[y][x][0] = '\0';
    }
  }
  int min_x = width, max_x = 0, min_y = height, max_y = 0;
  for (int i = 0; i < NUM_POINTS; i++) {
    if (points[i].pos.x < min_x)
      min_x = (int)points[i].pos.x;
    if (points[i].pos.x > max_x)
      max_x = (int)points[i].pos.x;
    if (points[i].pos.y < min_y)
      min_y = (int)points[i].pos.y;
    if (points[i].pos.y > max_y)
      max_y = (int)points[i].pos.y;
  }
  min_x -= 10;
  max_x += 10;
  min_y -= 5;
  max_y += 5;
  min_x = clamp(min_x, 0, width - 1);
  max_x = clamp(max_x, 0, width - 1);
  min_y = clamp(min_y, 0, height - 1);
  max_y = clamp(max_y, 0, height - 1);
  const char slime_chars[] = " .':~=+*#%@";
  int num_chars = strlen(slime_chars);
  for (int y = min_y; y <= max_y; y++) {
    for (int x = min_x; x <= max_x; x++) {
      float density = 0.0f;
      float dx = (float)x - center.pos.x;
      float dy = ((float)y - center.pos.y) * 2.0f;
      float dist_sq = dx * dx + dy * dy;
      if (dist_sq < 1.0f)
        dist_sq = 1.0f;
      density += 30.0f / dist_sq;
      for (int i = 0; i < NUM_POINTS; i++) {
        dx = (float)x - points[i].pos.x;
        dy = ((float)y - points[i].pos.y) * 2.0f;
        dist_sq = dx * dx + dy * dy;
        if (dist_sq < 0.1f)
          dist_sq = 0.1f;
        density += 22.0f / dist_sq;
      }
      density *= (1.0f + sinf(pulse_phase + (float)x * 0.1f) * 0.15f);
      if (density > METABALL_THRESHOLD) {
        float light_x = center.pos.x - RADIUS * 0.7f;
        float light_y = center.pos.y - RADIUS * 0.7f;
        dx = (float)x - light_x;
        dy = ((float)y - light_y) * 2.0f;
        dist_sq = dx * dx + dy * dy;
        float highlight = 1.0f / (1.0f + dist_sq * 0.008f);
        float glow = (density < 4.0f) ? 1.0f : 0.0f;
        int char_idx = clamp((int)(density * 0.55f), 0, num_chars - 1);
        if (y < 200 && x < 300) {
          buffer[y][x] = slime_chars[char_idx];
          const char *color = get_slime_color(density, highlight, glow);
          strncpy(color_buffer[y][x], color, 19);
        }
      }
    }
  }
  printf("\033[H");
  int help_height = 16;
  int slime_area_height = height - help_height;
  for (int y = 0; y < slime_area_height && y < 200; y++) {
    for (int x = 0; x < width && x < 300; x++) {
      if (use_color && color_buffer[y][x][0] != '\0') {
        printf("%s%c", color_buffer[y][x], buffer[y][x]);
      } else {
        putchar(buffer[y][x]);
      }
    }
    putchar('\n');
  }
  printf("\033[0m");
  printf("\033[90m");
  printf("WASD:Move E/C:Diag P/I:Pulse O:Oscil L/J/K:Rotate H/V:Squeeze "
         "+H/+V:Stretch U/Y/T/R:DirStretch\n");
  printf("B:Vibrate +B:Intense N:Wave SPC:Poke M:MultiPoke 1/2/3/4:Mode G:Grav "
         "F/+F:Wind +T:Turb\n");
  printf("C:Theme Z:Color X:Glow +Z:Hilite 0:Reset Q:Quit\n");
  printf("\033[0m");
  const char *mode_str[] = {"GENTLE", "NORMAL", "STRONG", "EXTREME"};
  const char *theme_str[] = {"GREEN", "PURPLE", "CYAN", "RAINBOW"};
  printf("Mode:%s Theme:%s Env:%s Glow:%s", mode_str[force_mode],
         theme_str[color_theme], env.enabled ? "ON" : "OFF",
         show_glow ? "ON" : "OFF");
  fflush(stdout);
}
void handle_input(char c) {
  switch (c) {
  case 'q':
    exit(0);
    break;
  case 'w':
    apply_force_global(vec_new(0, -2.0f));
    break;
  case 's':
    apply_force_global(vec_new(0, 2.0f));
    break;
  case 'a':
    apply_force_global(vec_new(-4.0f, 0));
    break;
  case 'd':
    apply_force_global(vec_new(4.0f, 0));
    break;
  case 'W':
    apply_force_global(vec_new(-2.0f, -2.0f));
    break;
  case 'E':
    apply_force_global(vec_new(2.0f, -2.0f));
    break;
  case 'X':
    apply_force_global(vec_new(-2.0f, 2.0f));
    break;
  case 'C':
    apply_force_global(vec_new(2.0f, 2.0f));
    break;
  case 'p':
    apply_radial_force(4.0f);
    break;
  case 'i':
    apply_radial_force(-4.0f);
    break;
  case 'o':
    apply_radial_force(sinf(pulse_phase) * 5.0f);
    break;
  case 'l':
    apply_rotation_force(0.8f);
    break;
  case 'j':
    apply_rotation_force(-0.8f);
    break;
  case 'k':
    apply_rotation_force(sinf(global_wobble * 2.0f) * 1.5f);
    break;
  case 'h':
    apply_squeeze(0);
    break;
  case 'v':
    apply_squeeze(1);
    break;
  case 'H':
    apply_stretch(0);
    break;
  case 'V':
    apply_stretch(1);
    break;
  case 'b':
    apply_vibration(3.0f);
    break;
  case 'B':
    apply_vibration(6.0f);
    break;
  case 'n':
    apply_wave(global_wobble);
    break;
  case 'u':
    apply_directional_stretch(0, 2.0f);
    break;
  case 'y':
    apply_directional_stretch(M_PI, 2.0f);
    break;
  case 't':
    apply_directional_stretch(M_PI / 2.0f, 2.0f);
    break;
  case 'r':
    apply_directional_stretch(-M_PI / 2.0f, 2.0f);
    break;
  case ' ':
    poke_random();
    break;
  case 'm':
    for (int i = 0; i < 5; i++)
      poke_random();
    break;
  case '1':
    force_mode = MODE_GENTLE;
    break;
  case '2':
    force_mode = MODE_NORMAL;
    break;
  case '3':
    force_mode = MODE_STRONG;
    break;
  case '4':
    force_mode = MODE_EXTREME;
    break;
  case 'g':
    env.enabled = !env.enabled;
    env.gravity = env.enabled ? GRAVITY_STRENGTH : 0.0f;
    break;
  case 'f':
    env.wind_x = (env.wind_x > 0) ? 0 : WIND_STRENGTH;
    env.enabled = 1;
    break;
  case 'F':
    env.wind_x = (env.wind_x < 0) ? 0 : -WIND_STRENGTH;
    env.enabled = 1;
    break;
  case 'T':
    env.turbulence = (env.turbulence > 0) ? 0 : 0.5f;
    env.enabled = 1;
    break;
  case 'c':
    color_theme = (color_theme + 1) % 4;
    break;
  case 'z':
    use_color = !use_color;
    break;
  case 'x':
    show_glow = !show_glow;
    break;
  case 'Z':
    show_highlights = !show_highlights;
    break;
  case '0':
    init_blob();
    break;
  }
}
void print_help() {
  printf("\033[2J\033[H");
  printf("Slime Blob Simulator\n");
  printf("Press any key to start\n");
  fflush(stdout);
  while (!kbhit())
    usleep(10000);
  getchar();
}
int main() {
  srand(time(NULL));
  get_term_size();
  enableRawMode();
  print_help();
  init_blob();
  printf("\033[2J");
  struct timespec last_time, current_time;
  clock_gettime(CLOCK_MONOTONIC, &last_time);
  while (1) {
    if (kbhit()) {
      char c = getchar();
      handle_input(c);
    }
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    float dt = (current_time.tv_sec - last_time.tv_sec) +
               (current_time.tv_nsec - last_time.tv_nsec) / 1000000000.0f;
    last_time = current_time;
    update_physics(dt);
    render();
    usleep(33000);
  }
  return 0;
}
