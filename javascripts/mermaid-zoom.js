// Click a rendered Mermaid diagram to enlarge it in a full-screen overlay.
//
// Material for MkDocs renders Mermaid into a *closed* shadow root, so the inner
// <svg> is unreachable from outside (querySelector/clone both fail). Instead we
// move the host element itself into an overlay and scale it with a CSS transform
// (which visually scales the shadow content), then restore it on close.
(function () {
  "use strict";

  var active = null; // { overlay, host, placeholder, prevStyle }

  function onKey(e) {
    if (e.key === "Escape") {
      close();
    }
  }

  function close() {
    if (!active) {
      return;
    }
    active.host.classList.remove("mermaid--zoomed");
    if (active.prevStyle === null) {
      active.host.removeAttribute("style");
    } else {
      active.host.setAttribute("style", active.prevStyle);
    }
    active.placeholder.replaceWith(active.host); // back to its original spot
    active.overlay.remove();
    document.body.style.overflow = "";
    document.removeEventListener("keydown", onKey);
    active = null;
  }

  function openZoom(host) {
    var overlay = document.createElement("div");
    overlay.className = "mermaid-lightbox";
    overlay.addEventListener("click", close);

    var button = document.createElement("button");
    button.className = "mermaid-lightbox__close";
    button.setAttribute("aria-label", "Close enlarged diagram");
    button.textContent = "×";
    button.addEventListener("click", function (e) {
      e.stopPropagation();
      close();
    });
    overlay.appendChild(button);

    var placeholder = document.createComment("mermaid-zoom-placeholder");
    var prevStyle = host.getAttribute("style"); // null if it had none
    host.replaceWith(placeholder);
    host.classList.add("mermaid--zoomed");
    overlay.appendChild(host);
    document.body.appendChild(overlay);
    document.body.style.overflow = "hidden";

    // Measure once laid out in the overlay, then scale to fit the viewport.
    var rect = host.getBoundingClientRect();
    if (rect.width && rect.height) {
      var scale = Math.min(
        (window.innerWidth * 0.96) / rect.width,
        (window.innerHeight * 0.92) / rect.height
      );
      host.style.transform = "scale(" + scale + ")";
    }

    active = { overlay: overlay, host: host, placeholder: placeholder, prevStyle: prevStyle };
    document.addEventListener("keydown", onKey);
  }

  document.addEventListener("click", function (e) {
    if (active) {
      return; // overlay handles closing
    }
    var host = e.target.closest(".mermaid");
    if (!host || host.querySelector("code")) {
      return; // not a rendered diagram (still raw source)
    }
    openZoom(host);
  });
})();
