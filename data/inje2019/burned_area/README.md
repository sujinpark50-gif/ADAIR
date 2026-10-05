# 2019 인제 산불 피해지역 참조 데이터

이 디렉터리는 **2019년 인제 산불**의 산불 확산 시뮬레이션 결과를 검증하기 위한 피해지역 참조 데이터를 포함한다.

## 파일

### `inje_2019_burn_reference_binary.tif`

산불 확산 모델과 직접 비교하기 위해 프로젝트 환경 격자에 맞춘 이진(binary) 참조 래스터이다.

- CRS: EPSG:5186
- 해상도: 90 m × 90 m
- Grid: 308 × 236
- `0`: 비피해지역 (Unburned)
- `1`: 피해지역 (Burned)
- 래스터 기준 피해면적: 약 344.25 ha

CA 산불 확산 시뮬레이션 결과와 동일한 격자에서 IoU, Dice 등의 공간 비교 지표를 계산하는 데 사용한다.

### `burn_perimeter_close50_5186.gpkg`

Sentinel-2 위성영상으로부터 추정한 산불 피해지역 경계 벡터이다.

- CRS: EPSG:5186
- 추정 면적: 약 345.13 ha
- `inje_2019_burn_reference_binary.tif` 생성에 사용

## 데이터 생성 과정

Sentinel-2 산불 전·후 영상을 이용하여 다음 과정으로 생성했다.

1. NBR 계산
2. dNBR 계산
3. dNBR ≥ 0.10 기준으로 burn candidate 추출
4. NDWI를 이용한 수역 (강) 제거
5. Sieve를 이용한 소규모 노이즈 제거
6. 주요 burn cluster 추출
7. Morphological closing (`+50 m → Dissolve → -50 m`) 적용
8. EPSG:5186으로 재투영
9. 프로젝트의 90 m 환경 격자에 맞춰 binary raster로 변환

## 주의사항

본 데이터는 공식 산불 경계 데이터가 아니다.

공식 발표 산림 피해면적 약 345 ha를 참고하여 50 m morphological closing을 적용한 보정된 위성영상 기반 참조 데이터(calibrated satellite-derived reference)이다.

따라서 실제 관측 경계 자체가 아니라 산불 확산 모델의 실험적 calibration 및 spatial validation을 위한 참조 데이터로 사용한다.