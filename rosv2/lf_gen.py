def generate_launch_file(num_turtles, speed=2.0):
    with open("turtle_launch_speed5.launch", "w") as file:
        file.write('<launch>\n')

        file.write(
            '  <node pkg="turtlesim" exec="turtlesim_node" name="turtlesim_node1"/>\n')

        file.write(
            f'  <node pkg="turtle_follower" exec="spawner_node" name="turtle_generator">\n')
        file.write(f'    <param name="turtles_count" value="{
                   num_turtles}"/>\n')
        file.write('  </node>\n')

        for i in range(1, num_turtles + 1):
            file.write(
                f'  <node pkg="turtle_follower" exec="follower_node" name="follower_node{i}">\n')
            file.write(f'    <param name="speed" value="{speed}"/>\n')
            file.write(f'    <param name="index" value="{i}"/>\n')
            file.write('  </node>\n')

        file.write('</launch>')


generate_launch_file(5, speed=1.0)
