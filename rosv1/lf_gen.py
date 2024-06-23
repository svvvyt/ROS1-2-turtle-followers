def generate_launch_file(num_turtles):
    with open("lf50.launch", "w") as file:
        file.write('<launch>\n')

        file.write(f'  <param name="turtles_count" value="{num_turtles}" />\n\n')
        file.write(
            '  <node pkg="turtlesim" type="turtlesim_node" name="simulator"/>\n')
        file.write(
            f'  <node pkg="turtlesim" type="turtle_teleop_key" name="input_handling"/>\n\n')

        file.write(
            f'  <node pkg="follower" type="spawner.py" name="spawner_node" />\n\n')

        for i in range(1, num_turtles + 1):
            file.write(
                f'  <node pkg="follower" type="follower.py" name="follower_node{i}">\n')
            file.write(f'    <param name="this_turtle_id" value="{i+1}"/>\n')
            file.write(f'    <param name="follow_turtle_id" value="{i}"/>\n')
            file.write('  </node>\n')

        file.write('</launch>')


# Пример использования
generate_launch_file(50)
