import os
import shutil
import math

# =========================================================================
# ⚠️ 경로 설정
# 원본 파일이 있는 폴더 (경로의 끝에 '\'는 붙이지 마세요.)
SOURCE_DIR = r"C:\Dev\KAIROS_Project_1\training\Right_training"
# 분할된 폴더들이 저장될 상위 폴더 (Right_sep)
TARGET_BASE_DIR = os.path.join(SOURCE_DIR, "Right_sep")
# 파일 묶음 개수
FILES_PER_FOLDER = 50
# =========================================================================

def split_files_into_folders():
    """
    SOURCE_DIR의 파일을 50개씩 묶어 TARGET_BASE_DIR의 하위 폴더로 이동시킵니다.
    폴더 이름은 Left_training50, Left_training100, ... 형식입니다.
    """
    
    # 1. 파일 목록 가져오기 및 정렬
    # 숨김 파일을 제외하고 파일만 가져옵니다.
    files = [f for f in os.listdir(SOURCE_DIR) if os.path.isfile(os.path.join(SOURCE_DIR, f))]
    
    # 파일명 기준으로 정렬 (숫자 순서대로 분할하기 위함)
    files.sort()
    
    total_files = len(files)
    if total_files == 0:
        print(f"경고: {SOURCE_DIR}에 파일이 없습니다.")
        return

    # 2. 대상 폴더 생성
    if not os.path.exists(TARGET_BASE_DIR):
        os.makedirs(TARGET_BASE_DIR)
        print(f"✅ 대상 폴더 생성: {TARGET_BASE_DIR}")
    
    print(f"총 {total_files}개의 파일을 {FILES_PER_FOLDER}개씩 분할합니다.")
    
    # 3. 파일 이동 작업 수행
    for i in range(0, total_files, FILES_PER_FOLDER):
        # 현재 묶음의 시작 인덱스와 끝 인덱스
        start_index = i
        end_index = i + FILES_PER_FOLDER
        
        # 새 폴더 이름 계산 (50, 100, 150...)
        folder_suffix = end_index 
        
        # 폴더 이름 정의
        new_folder_name = f"Right_training{folder_suffix}"
        target_folder = os.path.join(TARGET_BASE_DIR, new_folder_name)
        
        # 해당 폴더 생성
        if not os.path.exists(target_folder):
            os.makedirs(target_folder)
        
        # 현재 묶음에 포함될 파일 리스트 슬라이싱
        files_to_move = files[start_index:end_index]
        
        # 파일 이동
        for filename in files_to_move:
            source_file_path = os.path.join(SOURCE_DIR, filename)
            target_file_path = os.path.join(target_folder, filename)
            
            try:
                # 파일을 이동 (move)합니다. 복사(copy)를 원하시면 shutil.copy를 사용하세요.
                shutil.move(source_file_path, target_file_path)
            except Exception as e:
                print(f"❌ 파일 이동 중 오류 발생: {filename}. 오류: {e}")

    print("-" * 40)
    print(f"🎉 파일 분할 및 이동 완료. 총 {math.ceil(total_files / FILES_PER_FOLDER)}개의 폴더가 생성되었습니다.")

if __name__ == "__main__":
    split_files_into_folders()