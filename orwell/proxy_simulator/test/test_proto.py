import orwell.proxy_simulator.version1_pb2 as protobuff
import orwell.messages.controller_pb2 as pb_controller


def old_test():
    login = protobuff.login_message()
    login.client_id = 'toto'
    login.wished_robot_type = 'TANK'
    login_msg = login.SerializeToString()
    login_2 = protobuff.login_message()
    login_2.ParseFromString(login_msg)
    assert(login == login_2)


def test_input():
    message = pb_controller.Input()
    message.move.left = 0.2
    message.move.right = -0.5
    message.fire.weapon1 = False
    message.fire.weapon2 = True
    payload = message.SerializeToString()
    message2 = pb_controller.Input()
    message2.ParseFromString(payload)
    assert(message2.move.left == 0.2)
    assert(message2.move.right == -0.5)
    assert(not message2.fire.weapon1)
    assert(message2.fire.weapon2)


def main():
    old_test()
    test_input()

if ('__main__' == __name__):
    main()
