import {
  Scene,
  Points,
  Color,
  PerspectiveCamera,
  Camera,
  WebGLRenderer,
  PCDLoader,
  Texture,
  PointsMaterial,
  Euler,
  IntType,
  Vector3,
  Vector2
} from "three";

export default class PointCloudViewer {
  private scene: Scene;
  private camera: Camera;
  private renderer: WebGLRenderer;
  private container: HTMLElement;
  private points: Points;
  private dragStart: Vector2;
  private keysdown: { w: boolean; a: boolean; s: boolean; d: boolean; plus: boolean; minus: boolean };
  private mouseDown: boolean;

  constructor(el: HTMLElement) {
    this.container = el;
    this.dragStart = new Vector2(0, 0);
    this.mouseDown = false;
    this.init();
    this.animate();
    this.keysdown = { w: false, a: false, s: false, d: false, plus: false, minus: false };
  }

  init() {
    this.scene = new Scene();
    this.scene.background = new Color(0x0000000);

    this.camera = new PerspectiveCamera(15, window.innerWidth / window.innerHeight, 0.01, 1000000);
    this.camera.position.x = -0.4;
    this.camera.position.y = 10;
    this.camera.position.z = 47 + 106 * 6;
    this.camera.up.set(0, 0, 1);

    this.scene.add(this.camera);

    this.renderer = new WebGLRenderer({ antialias: true });

    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.container.appendChild(this.renderer.domElement);
    //adding something big isshh
    var loader = new PCDLoader();
    loader.load("./snapshot_05_08_20_05_36.pcd", (p: any) => {
      this.points = p;
      (this.points.material as PointsMaterial).size = 1;
      var center = this.points.geometry.boundingSphere.center;
      var radius = this.points.geometry.boundingSphere.radius;
      this.camera.position.x = center.x;
      this.camera.position.y = center.y;
      this.camera.position.z = center.z + radius * 8;
      this.scene.add(this.points);
    });

    window.onkeydown = e => {
      if (e.key === "w") this.keysdown.w = true;
      if (e.key === "a") this.keysdown.a = true;
      if (e.key === "s") this.keysdown.s = true;
      if (e.key === "d") this.keysdown.d = true;
      if (e.key === "+") this.keysdown.plus = true;
      if (e.key === "=") this.keysdown.plus = true;
      if (e.key === "-") this.keysdown.minus = true;
    };

    window.onkeyup = e => {
      if (e.key === "w") this.keysdown.w = false;
      if (e.key === "a") this.keysdown.a = false;
      if (e.key === "s") this.keysdown.s = false;
      if (e.key === "d") this.keysdown.d = false;
      if (e.key === "+") this.keysdown.plus = false;
      if (e.key === "=") this.keysdown.plus = false;
      if (e.key === "-") this.keysdown.minus = false;
    };
    window.onwheel = e => {
      this.camera.position.z += e.deltaY * this.points.geometry.boundingSphere.radius * 0.01;
    };
    window.onmousedown = e => {
      this.dragStart = new Vector2(e.x, e.y);
      this.mouseDown = true;
      this.renderer.domElement.style.cursor = "none !important";
    };
    window.onmousemove = e => {
      let speed = 0.25;
      if (!this.mouseDown) return;

      if (e.x != 0 && e.y != 0) {
        this.camera.position.x -= (e.x - this.dragStart.x) * speed;
        this.camera.position.y += (e.y - this.dragStart.y) * speed;

        this.dragStart = new Vector2(e.x, e.y);
      }
      window.onmouseup = e => {
        this.mouseDown = false;
        this.renderer.domElement.style.cursor = "initial ";
      };
    };

    setInterval(
      (() => {
        //movement and rotation
        let speed = 0.1;
        if (this.keysdown.w) this.points.rotateOnWorldAxis(new Vector3(1, 0, 0), -speed);
        if (this.keysdown.s) this.points.rotateOnWorldAxis(new Vector3(1, 0, 0), speed);
        if (this.keysdown.a) this.points.rotateOnWorldAxis(new Vector3(0, 1, 0), -speed);
        if (this.keysdown.d) this.points.rotateOnWorldAxis(new Vector3(0, 1, 0), speed);
        if (this.keysdown.plus) (this.points.material as PointsMaterial).size += speed;
        if (this.keysdown.minus) (this.points.material as PointsMaterial).size -= speed;
      }).bind(this),
      50
    );
  }

  animate() {
    requestAnimationFrame(this.animate.bind(this));

    this.renderer.render(this.scene, this.camera);
  }
}
