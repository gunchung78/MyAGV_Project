import cv2
import os

def apply_horizontal_flip_augmentation(input_folder="Ori_Img", output_folder="Left_Img"):
    """
    Ori_Img 폴더 내의 모든 PNG 이미지에 좌우 대칭(플립) Augmentation을 적용하고
    Augmented_Img 폴더에 저장합니다.
    """
    # 입력 폴더 존재 여부 확인
    if not os.path.exists(input_folder):
        print(f"오류: '{input_folder}' 폴더를 찾을 수 없습니다. 폴더를 생성하고 원본 이미지를 넣어주세요.")
        return

    # 출력 폴더 생성 (없으면 생성)
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        print(f"'{output_folder}' 폴더를 생성했습니다.")

    # 입력 폴더 내의 모든 파일 목록 가져오기
    image_files = [f for f in os.listdir(input_folder) if f.lower().endswith('.png')]

    if not image_files:
        print(f"'{input_folder}' 폴더에 PNG 이미지가 없습니다.")
        return

    print(f"'{input_folder}'에서 {len(image_files)}개의 PNG 이미지를 발견했습니다.")

    for filename in image_files:
        img_path = os.path.join(input_folder, filename)
        
        # 이미지 읽기
        img = cv2.imread(img_path)

        if img is None:
            print(f"경고: '{filename}' 이미지를 읽을 수 없습니다. 건너뜁니다.")
            continue

        # 좌우 대칭(플립) 적용 (flipCode = 1은 좌우 대칭)
        flipped_img = cv2.flip(img, 1)

        # 저장할 파일 이름 생성 (예: original_image.png -> original_image_flipped.png)
        base_name, ext = os.path.splitext(filename)
        flipped_filename = f"{base_name}_flipped{ext}"
        flipped_img_path = os.path.join(output_folder, flipped_filename)

        # 이미지 저장
        cv2.imwrite(flipped_img_path, flipped_img)
        # print(f"'{filename}' -> '{flipped_filename}' (좌우 반전) 저장 완료.")

    print(f"\n모든 이미지에 좌우 대칭 Augmentation을 적용하고 '{output_folder}'에 저장했습니다.")
    print(f"총 {len(image_files)}장의 원본 이미지에서 {len(image_files)}장의 반전 이미지를 생성했습니다.")

if __name__ == "__main__":
    apply_horizontal_flip_augmentation()