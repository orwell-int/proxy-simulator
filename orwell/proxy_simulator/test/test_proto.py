import orwell.proxy_simulator.version1_pb2 as protobuff


def main():
    login = protobuff.login_message()
    login.client_id = 'toto'
    login.wished_robot_type = 'TANK'
    login_msg = login.SerializeToString()
    login_2 = protobuff.login_message()
    login_2.ParseFromString(login_msg)
    assert(login == login_2)

if ('__main__' == __name__):
    main()
