from arachne_control import ArachneController

if __name__ == "__main__":
    controller = ArachneController(debug=False)
    controller.start_ps4_ctrl()  # could consider a different method of control?
