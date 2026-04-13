import polars as pl
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report
from micromlgen import port

# 1. 데이터 로드 및 전처리 (자네가 만든 로직 적용)
def get_train_data(file_path):
    df = pl.read_csv(file_path)
    df.columns = [c.lower() for c in df.columns] # 대소문자 통일
    
    # 정수형(Fixed-point) 변환 - 우노 최적화용
    processed = df.with_columns([
        (pl.col("ph") * 100).cast(pl.Int16).alias("ph_fixed"),
        (pl.col("turbidity")).cast(pl.Int16).alias("turb_fixed"),
        (pl.col("temperature")).cast(pl.Int16).alias("temp_fixed"),
        (pl.col("conductivity")).cast(pl.Int16).alias("cond_fixed")
    ])
    return processed.drop_nulls()

# 데이터 준비
data = get_train_data('water_quality1.csv')
X = data.select(['ph_fixed', 'turb_fixed', 'temp_fixed', 'cond_fixed']).to_numpy()
y = data.select('label').to_numpy().ravel()

# 2. 데이터 분할 (학습용 80%, 테스트용 20%)
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# 3. 모델 정의 (우노의 메모리를 고려해 나무의 깊이를 제한하네!)
# max_depth가 너무 깊으면 아두이노 코드 용량이 커지니 주의하게.
model = RandomForestClassifier(n_estimators=10, max_depth=5, random_state=42)
model.fit(X_train, y_train)

# 4. 성능 확인
y_pred = model.predict(X_test)
print(f"모델 정확도: {accuracy_score(y_test, y_pred):.2f}")
print("\n[상세 보고서]")
print(classification_report(y_test, y_pred))


# 학습된 model을 C++ 코드로 변환
c_code = port(model)
with open("model.h", "w") as f:
    f.write(c_code)

print("축하하네! model.h 파일이 생성되었어. 이제 아두이노에 넣기만 하면 되네!")