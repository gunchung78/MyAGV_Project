import cv2
import os
import numpy as np

# ----------------------------------------------------------------------
# 1. 설정 변수
# ----------------------------------------------------------------------

# 타겟 폴더 (경로 확인 필수)
TARGET_FOLDER = "./test_img/1"

# 적용할 밝기 계수 목록 (Factor). 1.0은 원본과 동일합니다.
TARGET_FACTORS = [0.9, 1.1, 1.2]

# ----------------------------------------------------------------------
# 2. 밝기 조정 함수 (HSV 기반)
# ----------------------------------------------------------------------

def adjust_brightness_in_hsv(image, brightness_factor):
    """
    HSV 색 공간의 V(Value) 채널을 조정하여 밝기를 변경합니다.
    """
    if image is None:
        return None
        
    # 1. BGR -> HSV 변환
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    
    # 2. V(Value) 채널 조정: 팩터를 곱하고 0~255 범위로 클리핑
    v_new = np.clip(v * brightness_factor, 0, 255).astype(np.uint8)
    
    # 3. 채널 병합 및 HSV -> BGR 재변환
    hsv_new = cv2.merge([h, s, v_new])
    adjusted_img = cv2.cvtColor(hsv_new, cv2.COLOR_HSV2BGR)
    return adjusted_img

# ----------------------------------------------------------------------
# 3. 배치 처리 실행 (모든 원본에 모든 팩터 적용)
# ----------------------------------------------------------------------

def apply_fixed_brightness_augmentation(target_folder=TARGET_FOLDER, factors=TARGET_FACTORS):
    # 타겟 폴더 존재 여부 확인
    if not os.path.exists(target_folder):
        print(f"❌ 오류: 폴더 '{target_folder}'를 찾을 수 없습니다. 경로를 확인해주세요.")
        return

    # 폴더 내의 모든 PNG 파일 목록 가져오기
    # image_files = [f for f in os.listdir(target_folder) if f.lower().endswith('.png')]
    # 폴더 내의 모든 JPG 파일 목록 가져오기
    image_files = [f for f in os.listdir(target_folder) if f.lower().endswith('.jpg')]
    
    print(f"🔍 '{target_folder}'에서 {len(image_files)}개의 PNG 원본 이미지를 대상으로 고정 밝기 증강을 시작합니다.")
    
    total_generated = 0

    for filename in image_files:
        img_path = os.path.join(target_folder, filename)
        
        # 이미지 읽기
        original_img = cv2.imread(img_path)
        
        if original_img is None: continue

        base_name, ext = os.path.splitext(filename)

        # 설정된 팩터별로 밝기 조정 및 저장
        for factor in factors:
            # 밝기 변환 적용 (1.0일 때는 변환 없이 저장)
            adjusted_img = adjust_brightness_in_hsv(original_img, factor)
            
            # 저장할 파일 이름 생성 (원본 이름_factor_0_7.png)
            # 파일 이름에 소수점 한 자리까지 팩터 값을 포함하여 저장
            factor_str = f"{factor:.1f}".replace('.', '_') # 1.1 -> 1_1
            new_filename = f"{base_name}_factor_{factor_str}{ext}"
            new_img_path = os.path.join(target_folder, new_filename)
            
            # 파일 저장
            # 새 파일로 저장되므로 원본 백업이 필요합니다. (사용자가 이미 백업 완료)
            cv2.imwrite(new_img_path, adjusted_img)
            total_generated += 1

    print("\n-------------------------------------------------")
    print(f"🎉 고정 밝기 Augmentation 완료!")
    print(f"원본 이미지 수: {len(image_files)}장")
    print(f"생성된 고정 밝기 변환 이미지 수: {total_generated}장")
    print(f"결과는 '{target_folder}' 폴더에 추가되었습니다.")
    print("-------------------------------------------------")

# 스크립트 실행
apply_fixed_brightness_augmentation()