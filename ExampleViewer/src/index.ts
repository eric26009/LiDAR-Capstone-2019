import PointCloudViewer from "./PointCloudViewer";
window.onload = () => {
  console.log("im in onload");
  let container = document.createElement("div");
  new PointCloudViewer(container);
  document.body.appendChild(container);
};
