import os 
import xml.etree.ElementTree as ET
from inertia import get_inertia

#Consts
width = 0.133
height = 0.133

def create_new_sdf(length_a, length_b, length_c, length_d, length_e):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sdf_path = os.path.join(current_dir, '..', 'models', 'four_more.sdf')
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    length_f = length_c + length_d

    for link in root.findall('.//link'):
        match link.get('name'):
            case 'A':
                update_link(link, length_a)
            case 'B':
                update_link(link, length_b)
                pose_elem = link.find('pose')
                if pose_elem is not None:
                    pose_elem.text = f'{length_a} 0 0 0 0 0'
            case 'C':
                update_link(link, length_c)
            case 'D':
                update_link(link, length_d, -1)
            case 'E':
                update_link(link, length_e, -1)
                pose_elem = link.find('pose')
                if pose_elem is not None:
                    pose_elem.text = f'-{length_d} 0 0 0 0 0'
            case 'F':
                update_link(link, length_f, -1)
                pose_elem = link.find('pose')
                if pose_elem is not None:
                    pose_elem.text = f'-{length_e} 0 0 0 -1.22173 0'
            case _:
                print('No match found')
                continue
            
    for frame in root.findall('.//frame'):
        name = frame.get("name")
        pose_elem = frame.find('pose')
        if pose_elem is not None:
            match name:
                case 'Bf_bushing':
                    pose_elem.text = f'{length_b - length_e} 0 0 -1.57079632679 0 0'
                case 'Fb_bushing':
                    pose_elem.text = f'-{length_f} 0 0 -1.57079632679 0 0'
                case 'Bc_bushing':
                    pose_elem.text = f'{length_b} 0 0 -1.57079632679 0 0'
                case 'Cb_bushing':
                    pose_elem.text = f'{length_c} 0 0 -1.57079632679 0 0'
                    
    output_path = os.path.join(current_dir, '..', 'models', 'four_more_iterated.sdf')
    tree.write(output_path)
    print('New SDF created at: ', output_path)

def update_link(link, length, sign=1):
    inertia_value, mass_value = get_inertia(length)
    inertial = link.find('inertial')
    if inertial is not None:
        mass_elem = inertial.find('mass')
        if mass_elem is not None:
            mass_elem.text = str(mass_value)
        inertia_elem = inertial.find('inertia')
        if inertia_elem is not None:
            for tag, value in zip(['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'], inertia_value):
                elem = inertia_elem.find(tag)
                if elem is not None:
                    elem.text = str(value)
        pose_elem = inertial.find('pose')
        if pose_elem is not None:
            pose_elem.text = f'{sign * (length / 2)} 0 0 0 0 0'
    
    for visual in link.findall('visual'):
        name = visual.get("name", "")
        if "pivot" in name.lower():
            continue
        box = visual.find(".//geometry/box")
        if box is not None:
            size_elem = box.find('size')
            if size_elem is not None:
                size_elem.text = f'{length} {width} {height}'
            pose_elem = visual.find('pose')
            if pose_elem is not None:
                pose_elem.text = f'{sign * (length / 2)} 0 0 0 0 0'

if __name__ == '__main__':
    create_new_sdf(1.525, 1.5125 + 2.5125, 3.1037, 6.0035, 1.5125)