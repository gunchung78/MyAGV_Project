import os
import glob

# =========================================================================
# ⚠️ 주의: 이 경로를 정확히 확인하고 수정하세요.
folder_path = r"C:\Dev\KAIROS_Project_1\training\Left_training"
# =========================================================================

def rename_files_sequentially(directory):
    """
    주어진 디렉토리 내의 모든 파일을 1부터 시작하는 순차적인 숫자로 변경합니다.
    (확장자는 유지됩니다.)
    """
    print(f"[{directory}] 폴더 내의 파일 이름 변경을 시작합니다.")
    
    # 1. 디렉토리 내의 모든 파일 목록을 가져옵니다.
    # glob을 사용하여 경로의 모든 파일을 가져옵니다. (숨김 파일 무시)
    # 리스트 순서는 OS 및 파일 시스템에 따라 달라질 수 있지만, 일반적으로는 디스크에 기록된 순서입니다.
    files = [f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]
    
    # 파일 이름을 변경하기 전에 리스트를 정렬하는 것이 좋습니다.
    # 원본 파일명 기준 정렬
    files.sort() 
    
    total_files = len(files)
    print(f"총 {total_files}개의 파일이 감지되었습니다.")
    
    if total_files == 0:
        print("경로에 파일이 없습니다. 작업을 종료합니다.")
        return

    # 2. 파일 이름 변경 작업 수행
    for index, filename in enumerate(files, start=1):
        # 파일 이름과 확장자를 분리합니다.
        name, ext = os.path.splitext(filename)
        
        # 새 파일 이름은 순차적인 숫자입니다. (예: 1, 2, 3...)
        new_filename = str(index) + ext
        
        old_file = os.path.join(directory, filename)
        new_file = os.path.join(directory, new_filename)
        
        try:
            # 파일 이름 변경
            os.rename(old_file, new_file)
            # print(f"'{filename}' -> '{new_filename}'")
        except Exception as e:
            print(f"오류 발생: '{filename}' 파일을 '{new_filename}'으로 변경하지 못했습니다. 오류: {e}")

    print("-" * 30)
    print(f"✅ 파일 이름 변경 완료! 총 {total_files}개의 파일이 1부터 시작하는 숫자로 변경되었습니다.")
    print("스크립트 실행 전과 후의 파일 개수가 동일한지 확인하세요.")


if __name__ == "__main__":
    rename_files_sequentially(folder_path)