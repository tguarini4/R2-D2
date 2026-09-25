function sendCommand(direction) {
    fetch("/move", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ direction })
    });
}

document.addEventListener("keydown", (e) => {
    if (e.repeat) return;

    if (e.key === "ArrowUp") sendCommand("forward");
    if (e.key === "ArrowDown") sendCommand("backward");
    if (e.key === "ArrowLeft") sendCommand("left");
    if (e.key === "ArrowRight") sendCommand("right");
});

document.addEventListener("keyup", () => {
    sendCommand("stop");
});
