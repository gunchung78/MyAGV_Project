import os

# ⚠️ 1. 대상 폴더 경로 설정 (요청하신 경로)
TARGET_DIR = r'C:\Dev\KAIROS_Project_1\test_img\1'

def rename_files_in_folder(directory):
    """
    폴더 내의 파일 이름을 0001부터 오름차순으로 변경합니다.
    """
    print(f"✅ 대상 폴더: {directory}")
    
    # 1. 폴더 내용 리스트 가져오기
    # os.listdir()은 순서가 보장되지 않을 수 있으므로, sorted()를 사용하여 정렬
    # 일반적으로는 파일 시스템의 기본 정렬 순서(알파벳/숫자)를 따릅니다.
    files = sorted(os.listdir(directory))
    
    # 카운터를 1부터 시작 (0001...)
    counter = 1
    
    for filename in files:
        # 2. 전체 파일 경로 생성
        old_file_path = os.path.join(directory, filename)
        
        # 파일이 디렉토리가 아닌지 확인 (하위 폴더는 건너뜀)
        if os.path.isfile(old_file_path):
            # 3. 파일 이름과 확장자 분리
            # os.path.splitext()를 사용하여 확장자만 정확하게 추출
            _, file_extension = os.path.splitext(filename)
            
            # 4. 새 파일 이름 (0001, 0002...) 생성
            # {:04d}는 숫자를 4자리로 만들고, 앞을 0으로 채우라는 의미입니다.
            new_filename = f'{counter:04d}{file_extension}'
            new_file_path = os.path.join(directory, new_filename)
            
            # 5. 파일 이름 변경 실행
            try:
                os.rename(old_file_path, new_file_path)
                print(f"   [변경] {filename} -> {new_filename}")
                counter += 1
            except Exception as e:
                print(f"   [오류] {filename} 변경 실패: {e}")
                
    print(f"\n🎉 파일 이름 변경 완료. 총 {counter - 1}개 파일 처리됨.")

if __name__ == '__main__':
    if os.path.isdir(TARGET_DIR):
        rename_files_in_folder(TARGET_DIR)
    else:
        print(f"❌ 오류: 지정된 경로 '{TARGET_DIR}'가 존재하지 않거나 폴더가 아닙니다.")