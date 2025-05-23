#define _USE_MATH_DEFINES
#include <Novice.h>
#include <algorithm>
#include <cmath>
#include <imgui.h>

constexpr char kWindowTitle[] = "LE2B_20_ヘンミ_ハクト";
constexpr float kPI = 3.14159265358979323846f;

///------------------------- 基本構造体 -------------------------//
struct Vector3 {
  float x, y, z;
};
struct Matrix4x4 {
  float m[4][4];
};
struct Sphere {
  Vector3 center;
  float radius;
};
struct Plane {
  Vector3 normal;
  float distance;
}; // normal は正規化済み前提

///------------------------- ベクトル演算 -----------------------//
inline Vector3 Add(const Vector3 &a, const Vector3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline Vector3 Sub(const Vector3 &a, const Vector3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline Vector3 Scale(const Vector3 &v, float s) {
  return {v.x * s, v.y * s, v.z * s};
}
inline float Dot(const Vector3 &a, const Vector3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline float Len(const Vector3 &v) { return std::sqrt(Dot(v, v)); }
inline Vector3 Normalize(const Vector3 &v) {
  float l = Len(v);
  return l > 0.00001f ? Scale(v, 1.0f / l) : Vector3{0, 1, 0};
}
inline Vector3 Cross(const Vector3 &a, const Vector3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Vector3 Perpendicular(const Vector3 &n) { // 平面用
  return (std::abs(n.x) < 0.57735f) ? Vector3{1, 0, 0} : Vector3{0, 1, 0};
}

///------------------------- 衝突判定 ---------------------------//
inline bool IsCollision(const Sphere &s, const Plane &p) {
  float d = Dot(p.normal, s.center) - p.distance;
  return std::abs(d) <= s.radius;
}

///------------------------- 行列ユーティリティ -----------------//
Matrix4x4 I() {
  Matrix4x4 r{};
  for (int i = 0; i < 4; i++)
    r.m[i][i] = 1.0f;
  return r;
}
Matrix4x4 T(const Vector3 &t) {
  Matrix4x4 r = I();
  r.m[3][0] = t.x;
  r.m[3][1] = t.y;
  r.m[3][2] = t.z;
  return r;
}
Matrix4x4 Rx(float a) {
  Matrix4x4 r = I();
  r.m[1][1] = cosf(a);
  r.m[1][2] = sinf(a);
  r.m[2][1] = -sinf(a);
  r.m[2][2] = cosf(a);
  return r;
}
Matrix4x4 Ry(float a) {
  Matrix4x4 r = I();
  r.m[0][0] = cosf(a);
  r.m[0][2] = -sinf(a);
  r.m[2][0] = sinf(a);
  r.m[2][2] = cosf(a);
  return r;
}
Matrix4x4 Mul(const Matrix4x4 &A, const Matrix4x4 &B) {
  Matrix4x4 r{};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 4; k++)
        r.m[i][j] += A.m[i][k] * B.m[k][j];
  return r;
}
Vector3 Transform(const Vector3 &v, const Matrix4x4 &m) {
  Vector3 r;
  float w;
  r.x = v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0] + m.m[3][0];
  r.y = v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1] + m.m[3][1];
  r.z = v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2] + m.m[3][2];
  w = v.x * m.m[0][3] + v.y * m.m[1][3] + v.z * m.m[2][3] + m.m[3][3];
  r.x /= w;
  r.y /= w;
  r.z /= w;
  return r;
}
Matrix4x4 P(float fovY, float aspect, float n, float f) {
  Matrix4x4 r{};
  float s = 1.0f / tanf(fovY * 0.5f);
  r.m[0][0] = s / aspect;
  r.m[1][1] = s;
  r.m[2][2] = f / (n - f);
  r.m[2][3] = -1;
  r.m[3][2] = (n * f) / (n - f);
  return r;
}
Matrix4x4 Vp(float l, float t, float w, float h) {
  Matrix4x4 r{};
  r.m[0][0] = w * 0.5f;
  r.m[1][1] = h * 0.5f;
  r.m[3][0] = l + w * 0.5f;
  r.m[3][1] = t + h * 0.5f;
  r.m[3][3] = 1.0f;
  r.m[2][2] = 1.0f;
  return r;
}

///------------------------- 描画ヘルパ -------------------------//
void DrawGrid(const Matrix4x4 &vp, const Matrix4x4 &vm) {
  constexpr float half = 4.0f;
  constexpr int div = 20;
  float step = (half * 2) / div;
  for (int i = 0; i <= div; i++) {
    float o = -half + i * step;
    Vector3 a{o, 0, -half}, b{o, 0, half};
    a = Transform(Transform(a, vp), vm);
    b = Transform(Transform(b, vp), vm);
    Novice::DrawLine((int)a.x, (int)a.y, (int)b.x, (int)b.y, 0x444444FF);
    a = {-half, 0, o};
    b = {half, 0, o};
    a = Transform(Transform(a, vp), vm);
    b = Transform(Transform(b, vp), vm);
    Novice::DrawLine((int)a.x, (int)a.y, (int)b.x, (int)b.y, 0x444444FF);
  }
}
void DrawSphereWire(const Sphere &sp, const Matrix4x4 &vp, const Matrix4x4 &vm,
                    uint32_t col) {
  constexpr int latDiv = 12, lonDiv = 24;
  for (int lat = 0; lat <= latDiv; lat++) {
    float la = (-0.5f + float(lat) / latDiv) * kPI;
    for (int lo = 0; lo < lonDiv; lo++) {
      float a = 2 * kPI * float(lo) / lonDiv,
            b = 2 * kPI * float(lo + 1) / lonDiv;
      Vector3 p1{sp.radius * cosf(la) * cosf(a) + sp.center.x,
                 sp.radius * sinf(la) + sp.center.y,
                 sp.radius * cosf(la) * sinf(a) + sp.center.z};
      Vector3 p2{sp.radius * cosf(la) * cosf(b) + sp.center.x,
                 sp.radius * sinf(la) + sp.center.y,
                 sp.radius * cosf(la) * sinf(b) + sp.center.z};
      p1 = Transform(Transform(p1, vp), vm);
      p2 = Transform(Transform(p2, vp), vm);
      Novice::DrawLine((int)p1.x, (int)p1.y, (int)p2.x, (int)p2.y, col);
    }
  }
}
void DrawPlane(const Plane &pl, const Matrix4x4 &vp, const Matrix4x4 &vm,
               uint32_t col) {
  Vector3 c = Scale(pl.normal, pl.distance);
  Vector3 e1 = Normalize(Cross(pl.normal, Perpendicular(pl.normal)));
  Vector3 e2 = Cross(pl.normal, e1);
  constexpr float s = 5.0f;
  e1 = Scale(e1, s);
  e2 = Scale(e2, s);
  Vector3 p[4]{Add(c, Add(e1, e2)), Add(c, Sub(e1, e2)),
               Add(c, Sub(Scale(e1, -1), e2)),
               Add(c, Sub(Scale(e1, -1), Scale(e2, -1)))};
  for (int i = 0; i < 4; i++) {
    Vector3 a = Transform(Transform(p[i], vp), vm);
    Vector3 b = Transform(Transform(p[(i + 1) % 4], vp), vm);
    Novice::DrawLine((int)a.x, (int)a.y, (int)b.x, (int)b.y, col);
  }
}

//------------------------------------------------------------------
// WinMain
//------------------------------------------------------------------
int WINAPI WinMain(HINSTANCE hInst, HINSTANCE hPrev, LPSTR lpCmd, int nCmd) {
  (void)hInst;
  (void)hPrev;
  (void)lpCmd;
  (void)nCmd;

  Novice::Initialize(kWindowTitle, 1280, 720);
  char keys[256]{}, pre[256]{};

  // パラメータ
  Vector3 camPos{0, 2, -8};
  Vector3 camRot{0, 0, 0};
  Sphere sphere{{0, 1, 0}, 1.0f};
  Plane plane{{0, 1, 0}, 0.0f};

  int prevMX = 0, prevMY = 0;
  while (Novice::ProcessMessage() == 0) {
    Novice::BeginFrame();
    memcpy(pre, keys, 256);
    Novice::GetHitKeyStateAll(keys);

    //---------------- カメラ操作 ----------------
    constexpr float mv = 0.05f, rt = 0.005f;
    if (keys[DIK_W])
      camPos.z += mv;
    if (keys[DIK_S])
      camPos.z -= mv;
    if (keys[DIK_A])
      camPos.x -= mv;
    if (keys[DIK_D])
      camPos.x += mv;
    if (keys[DIK_UP])
      camPos.y += mv;
    if (keys[DIK_DOWN])
      camPos.y -= mv;
    int mx, my;
    Novice::GetMousePosition(&mx, &my);
    if (Novice::IsPressMouse(1)) {
      camRot.y += (mx - prevMX) * rt;
      camRot.x += (my - prevMY) * rt;
      camRot.x = std::clamp(camRot.x, -kPI * 0.49f, kPI * 0.49f);
    }
    prevMX = mx;
    prevMY = my;

    //---------------- ImGui ----------------
    ImGui::Begin("Control");
    ImGui::DragFloat3("Cam Pos", &camPos.x, 0.01f);
    ImGui::DragFloat3("Cam Rot", &camRot.x, 0.01f);
    ImGui::Separator();
    ImGui::DragFloat3("Sphere C", &sphere.center.x, 0.01f);
    ImGui::DragFloat("Sphere R", &sphere.radius, 0.01f, 0.01f);
    ImGui::Separator();
    ImGui::DragFloat3("Plane N", &plane.normal.x, 0.01f);
    ImGui::DragFloat("Plane D", &plane.distance, 0.01f);
    plane.normal = Normalize(plane.normal);
    bool hit = IsCollision(sphere, plane);
    ImGui::Text("Collision : %s", hit ? "YES" : "NO");
    ImGui::End();

    //---------------- 行列 ----------------
    Matrix4x4 view =
        Mul(Ry(-camRot.y),
            Mul(Rx(-camRot.x), T({-camPos.x, -camPos.y, -camPos.z})));
    Matrix4x4 proj = P(0.45f, 1280.0f / 720.0f, 0.1f, 100.0f);
    Matrix4x4 vp = Mul(view, proj);
    Matrix4x4 vm = Vp(0, 0, 1280, 720);

    //---------------- 描画 ----------------
    DrawGrid(vp, vm);
    DrawPlane(plane, vp, vm, 0xFFFFFFFF);
    DrawSphereWire(sphere, vp, vm, hit ? 0xFF4444FF : 0x4444FFFF);

    Novice::EndFrame();
    if (pre[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE])
      break;
  }
  Novice::Finalize();
  return 0;
}
