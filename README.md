# Algorithm-Project

알고리즘 10조 프로젝트입니다

## 현재 진행 상황

1. gpt를 통해서 엑셀 파일(.xlsl)을 .csv 파일로 바꿔서 읽는 데에 성공
2. SFML 외부 라이브러리를 통해서 노드와 간선 시각화 성공
   1. 더 좋은 시각화 라이브러리가 있다면 변경 검토
   2. vcpkg를 통해서 SFML 라이브러리를 다운받으시면 사용하실 수 있습니다
3. 현재는 플레이어, 탈출구, 불의 위치를 노드 중 하나에서 랜덤으로 정하고 있습니다. 플레이어는 빨간색, 탈출구는 파란색, 불은 자홍색, 플레이어->탈출구 루트는 빨간색 간선으로 표시됩니다.
4. 실시간 애니메이션 효과를 통해서 불의 확산과 플레이어의 이동을 나타내고 있습니다. 불의 확산되면 사용자는 현재 노드를 기준으로 다시 최단 경로를 계산합니다.
5. 현재 최단 경로 알고리즘으로 다이익스트라를 사용하고 있습니다.
