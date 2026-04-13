import polars as pl

def preprocessing_engine(df: pl.DataFrame) -> pl.DataFrame:
    """Polars-based preprocessing equivalent to the original pandas version.

    Steps:
    1. pH fixed-point (multiply by 100 and cast to integer)
    2. Sensor fusion feature (absolute diff between ToF and Sonic, as int)
    3. Rolling statistics (mean, std) over `ToF_mm` with a small window
    """
    cols = df.columns
    out = df

    # 1. Fixed-point transform for pH
    if 'ph' in cols:
        out = out.with_columns((pl.col('ph') * 100).cast(pl.Int64).alias('ph_fixed'))

    # 2. Sensor fusion feature (ToF - Sonic)
    if 'ToF_mm' in cols and 'Sonic_mm' in cols:
        out = out.with_columns(((pl.col('ToF_mm') - pl.col('Sonic_mm')).abs().cast(pl.Int64)).alias('diff_fixed'))

    # 3. Rolling statistics (window small because of microcontroller memory limits)
    window_size = 5
    if 'ToF_mm' in cols:
        # rolling_mean / rolling_std are available as Expr methods
        out = out.with_columns([
            pl.col('ToF_mm').rolling_mean(window_size).alias('dist_mean'),
            pl.col('ToF_mm').rolling_std(window_size).alias('dist_std'),
        ])

    # drop rows with any nulls (equivalent to pandas' dropna)
    return out.drop_nulls()


# 사용 예시 (Polars로 CSV 읽기)
df = pl.read_csv('water_quality1.csv')
# 1. 컬럼명 대소문자 문제 해결을 위해 전체 소문자화
df.columns = [c.lower() for c in df.columns]
processed_df = preprocessing_engine(df)

# 2. 데이터 건강검진 (시니어의 체크리스트)
print("--- 데이터셋 건강 검진 ---")
print(f"전체 데이터 개수: {len(processed_df)}")
print("\n[라벨 분포]")
print(processed_df['label'].value_counts())

print("\n[주요 피처 범위 - 정수 연산 가능 여부 확인]")
# 아두이노 int16_t 범위(-32,768 ~ 32,767)를 넘는지 체크
print(processed_df.select([
    pl.col("ph_fixed").max().alias("max_ph"),
    pl.col("turbidity").max().alias("max_turb"),
    pl.col("conductivity").max().alias("max_cond")
]))